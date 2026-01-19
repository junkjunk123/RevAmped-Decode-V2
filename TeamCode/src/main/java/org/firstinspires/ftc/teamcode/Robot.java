package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
import org.firstinspires.ftc.teamcode.mechanisms.lift.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.octocanum.Octocanum;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;
import org.firstinspires.ftc.teamcode.utils.AtomicReadOnce;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channels;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Notifier;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

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
    public final Octocanum octocanum;
    public final IntakeMotor intakeMotor;
    public final ColorManager intakeColor;
    public final IntakeDistance intakeDistance;
    private Lift lift;
    public final TableCompartmentManager tableCompartments;
    private final boolean teleop;
    private final List<LynxModule> hubs;
    private final HardwareMap hardwareMap;
    private CycleState robotState = CycleState.INTAKE;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Robot(HardwareMap hardwareMap, PathSupplier pathSupplier) {
        this.hardwareMap = hardwareMap;
        hubs = hardwareMap.getAll(LynxModule.class);
        setBulkReadMode(LynxModule.BulkCachingMode.MANUAL);
        drivetrain = pathSupplier != null ? new Drivetrain(hardwareMap, pathSupplier) : new Drivetrain(hardwareMap);
        teleop = pathSupplier == null;
        octocanum = teleop ? new Octocanum(hardwareMap) : null;
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intakeMotor = new IntakeMotor(hardwareMap);
        table = new Table(hardwareMap, Encoder.fromMotor(drivetrain.leftRear));
        popper = new Popper(hardwareMap);
        hood = new Hood(hardwareMap);
        intakeColor = new ColorManager(hardwareMap);
        //intakeDistance = new IntakeDistance(hardwareMap);
        intakeDistance = null;
        INSTANCE = this;
        RobotStateHandler.CycleState.DRIVE_TO_SHOOT.init(drivetrain.follower, turret, hood, flywheel, teleop);
        tableCompartments = new TableCompartmentManager(intakeColor);
        if (!teleop) Scheduler.getInstance().schedule(init());
    }

    public ICommand init() {
        return new Parallel(
                new Sequential(
                        table.reset(),
                        popper.neutral()
                ),
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
        //intakeDistance.update();
        hood.update();
        robotState.update();

        if (teleop) {
            octocanum.update();
            drivetrain.update();

            if (lift != null) lift.update();
        }
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
        AtomicReadOnce<Table.RelativeState> reader = table.pendingStateReader();
        return table.setState(() -> {
            if (Globals.randomizationState == null) return reader.read().ordinal();
            return tableCompartments.sort(reader.read().ordinal());}
        );
    }

    public ICommand shootAll() {
        return new Sequential(
                new Instant(intakeMotor::intake),
                table.fullRotation()
        );
    }

    public ICommand shootAll(Supplier<Double> delay) {
        AtomicReference<float[]> shootSequence = new AtomicReference<>();
        return new Conditional(
                () -> delay.get() < 10,
                shootAll(),
                new Sequential(
                        new Instant(() -> {
                            intakeMotor.intakeSlow();
                            shootSequence.set(table.getState().getShootStates());
                        }),
                        table.setPos(() -> shootSequence.get()[0]),
                        new Instant(intakeMotor::stop),
                        new Wait(delay.get()),
                        new Instant(intakeMotor::intakeSlow),
                        table.setPos(() -> shootSequence.get()[1]),
                        new Instant(intakeMotor::shooting),
                        new Wait(delay.get()),
                        table.setPos(() -> shootSequence.get()[2] + Table.FULL_REVOLUTION / 3),
                        new Instant(tableCompartments::removeAll)
                )
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
                table.reset(),
                new Instant(intakeMotor::intake),
                popper.neutral()
        );
    }

    public ICommand sortAndShoot() {
        return new Sequential(
                sort(),
                new Parallel(
                        turret.reached(),
                        popper.pop()
                ),
                shootAll(() -> Globals.allianceColor == null ? 0.0 : 175.0)
        );
    }

    public Notifier shootAndReset() {
        return new Notifier(c -> new Sequential(
                shootAll(),
                Channels.send(c, Channels::signal),
                new Parallel(
                        new Sequential(
                                resetTableAfterShooting(),
                                Channels.send(c, Channels::signal)
                        ),
                        new Sequential(
                                resetShooter(),
                                Channels.send(c, Channels::signal)
                        )
                )
        ));
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

    public ICommand lift() {
        return new Sequential(
                new Instant(() -> {
                    lift = new Lift(hardwareMap, Encoder.fromMotor(intakeMotor.get()));
                    intakeMotor.deenergize();
                    flywheel.deenergize();
                    popper.deenergize();
                    drivetrain.apply(DcMotorEx::setMotorDisable);
                    octocanum.apply(HwServo::deenergize);
                    table.deenergize();
                    hood.deenergize();
                }),
                new WaitUntil(turret::deenergized),
                lift.lift()
        );
    }
}
