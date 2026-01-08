package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.CycleState;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.DriveMessage;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.IntakeMessage;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.Message;
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
import org.firstinspires.ftc.teamcode.utils.AtomicReadOnce;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
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
        if (!teleop) schedule(init());
    }

    public CommandBuilder init() {
        return parallel(
                instant(hood::rest),
                popper.neutral(),
                table.reset(),
                turret.runToState(Turret.MoveState.PresetState.REST)
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

    public CommandBuilder sort() {
        AtomicReadOnce<Table.RelativeState> reader = table.pendingStateReader();
        return table.setState(() -> {
            if (Globals.randomizationState == null) return reader.read().ordinal();
            return tableCompartments.sort(reader.read().ordinal());}
        );
    }

    public CommandBuilder shootAll() {
        return sequential(
                instant(intakeMotor::intake),
                table.fullRotation()
        );
    }

    public CommandBuilder shootAll(Supplier<Double> delay) {
        AtomicReference<float[]> shootSequence = new AtomicReference<>();
        return conditional(
                () -> delay.get() < 10,
                shootAll(),
                sequential(
                        instant(() -> {
                            intakeMotor.intakeSlow();
                            shootSequence.set(table.getState().getShootStates());
                        }),
                        table.setPos(() -> shootSequence.get()[0]),
                        instant(intakeMotor::stop),
                        Commands.wait(delay.get()),
                        instant(intakeMotor::intakeSlow),
                        table.setPos(() -> shootSequence.get()[1]),
                        instant(intakeMotor::shooting),
                        Commands.wait(delay.get()),
                        table.setPos(() -> shootSequence.get()[2] + Table.FULL_REVOLUTION / 3),
                        instant(tableCompartments::removeAll)
                )
        );
    }

    public CommandBuilder shootAll(double delay) {
        if (delay < 10) return shootAll();
        return shootAll(() -> delay);
    }

    public CommandBuilder resetShooter() {
        return parallel(
                turret.resetTurret(),
                instant(() -> {
                    flywheel.stop();
                    hood.rest();
                })
        );
    }

    public CommandBuilder resetAfterShooting() {
        return parallel(
                resetShooter(),
                resetTableAfterShooting()
        );
    }

    public CommandBuilder resetTableAfterShooting() {
        return sequential(
                instant(intakeMotor::stop),
                table.reset(),
                Commands.wait(500.0),
                instant(intakeMotor::intake),
                popper.neutral()
        );
    }

    public CommandBuilder sortAndShoot() {
        return sequential(
                sort(),
                parallel(
                        turret.reached(),
                        popper.pop()
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
