package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.intake.ColorManager;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistance;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.intake.TableCompartmentManager;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.RobotStateHandler.CycleState;
import org.firstinspires.ftc.teamcode.RobotStateHandler.Message;
import org.firstinspires.ftc.teamcode.RobotStateHandler.DriveMessage;
import org.firstinspires.ftc.teamcode.RobotStateHandler.IntakeMessage;

import java.util.List;
import java.util.function.Supplier;

public class Robot {
    public static Robot INSTANCE;
    public final Drivetrain drivetrain;
    public final Turret turret;
    public final Flywheel flywheel;
    public final Table table;
    public final Hood hood;
    public final Popper popper;
    public final IntakeMotor intakeMotor;
    public final ColorManager intakeColor;
    public final IntakeDistance intakeDistance;
    public final TableCompartmentManager tableCompartments;
    private final boolean teleop;
    private final List<LynxModule> hubs;
    private CycleState robotState = CycleState.INTAKE;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Robot(HardwareMap hardwareMap, PathSupplier pathSupplier) {
        hubs = hardwareMap.getAll(LynxModule.class);
        setBulkReadMode(LynxModule.BulkCachingMode.MANUAL);
        drivetrain = pathSupplier != null ? new Drivetrain(hardwareMap, pathSupplier) : new Drivetrain(hardwareMap);
        teleop = pathSupplier == null;
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intakeMotor = new IntakeMotor(hardwareMap);
        table = new Table(hardwareMap, Encoder.external(intakeMotor.get()));
        popper = new Popper(hardwareMap);
        hood = new Hood(hardwareMap);
        intakeColor = new ColorManager(hardwareMap);
        intakeDistance = new IntakeDistance(hardwareMap);
        INSTANCE = this;
        RobotStateHandler.CycleState.DRIVE_TO_SHOOT.init(drivetrain.follower, turret, hood, flywheel, teleop);
        tableCompartments = new TableCompartmentManager(intakeColor);
        if (!teleop) Scheduler.getInstance().schedule(init());
    }

    public ICommand init() {
        return new Sequential(
                new Instant(() -> {
                    hood.rest();
                    popper.neutral();
                    turret.move(Turret.MoveState.PresetState.REST);
                }),
                table.reset()
        );
    }

    public void update() {
        clearBulkCache();
        intakeColor.update();
        flywheel.update();
        turret.update();
        intakeMotor.update();
        table.update();
        popper.update();
        intakeDistance.update();
        hood.update();
        robotState.update();
    }

    public void setBulkReadMode(LynxModule.BulkCachingMode mode) {
        for (LynxModule module : hubs) {
            module.setBulkCachingMode(mode);
        }
    }

    public void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    public boolean isTeleop() {
        return teleop;
    }

    public CycleState getRobotState() {
        return robotState;
    }

    public void setRobotState(Message message) {
        if (message instanceof DriveMessage driveState)
            CycleState.DRIVE_TO_SHOOT.INSTANCE = driveState.driveState;
        else if (message instanceof IntakeMessage intakeMessage)
            CycleState.INTAKE.INSTANCE = intakeMessage.intakeState;
        else if (message instanceof CycleState.Intake)
            CycleState.INTAKE.INSTANCE = RobotStateHandler.IntakeState.INTAKING;
        robotState = message.cycleState();
    }

    public ICommand sort() {
        return table.setState(() -> {if (Globals.randomizationState == null) return table.getState().ordinal();
            return tableCompartments.sort(table.getState().ordinal());}
        );
    }

    public ICommand shootAll() {
        return new Sequential(
                new Instant(intakeMotor::intake),
                table.fullRotation()
        );
    }

    public ICommand shootAll(Supplier<Double> delay) {
        float[] shootSequence = table.getState().getShootStates();
        assert shootSequence != null && shootSequence.length > 2;
        return new Sequential(
                new Instant(intakeMotor::intakeSlow),
                table.setState(shootSequence[0]),
                new Instant(intakeMotor::stop),
                new Wait(delay.get()),
                new Instant(intakeMotor::intakeSlow),
                table.setState(shootSequence[1]),
                new Instant(intakeMotor::shooting),
                new Wait(delay.get()),
                table.setState(shootSequence[2] + Table.FULL_REVOLUTION / 3),
                new Instant(tableCompartments::removeAll)
        );
    }

    public ICommand shootAll(double delay) {
        if (delay < 10) return shootAll();
        return shootAll(() -> delay);
    }

    public ICommand resetShooter() {
        return new Parallel(
                turret.resetTurret(),
                new Instant(() -> {
                    flywheel.stop();
                    hood.rest();
                })
        );
    }

    public ICommand resetAfterShooting() {
        return new Parallel(
                resetShooter(),
                resetTableAfterShooting()
        );
    }

    public ICommand resetTableAfterShooting() {
        return new Sequential(
                new Instant(intakeMotor::stop),
                new Race(
                        table.reset(),
                        new Wait(1000)
                ),
                new Wait(500),
                new Instant(() -> {
                    popper.neutral();
                    intakeMotor.intake();
                })
        );
    }

    public ICommand sortAndShoot() {
        return new Sequential(
                sort(),
                new Instant(popper::pop),
                new Parallel(
                        new WaitUntil(turret::reached),
                        new Wait(250)
                ),
                shootAll(() -> Globals.allianceColor == null ? 0.0 : 175.0)
        );
    }

    public void shootNear() {
        hood.near();
        flywheel.near();
    }

    public void shootFar() {
        hood.far();
        flywheel.far();
    }

    public void shootMedium() {
        hood.medium();
        flywheel.medium();
    }
}
