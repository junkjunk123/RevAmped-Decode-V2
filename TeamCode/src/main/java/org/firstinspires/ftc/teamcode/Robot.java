package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Sequential;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.calc.Vector2D;
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
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretState;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerColorSensors;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeLimelight;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channels;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Notifier;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class Robot {
    public static Robot INSTANCE;
    public final Drivetrain drivetrain;
    public final ServoTurret turret;
    public final Flywheel flywheel;
    public final Table table;
    public final Hood hood;
    public final Popper popper;
    public final IntakeMotor intakeMotor;
    public final SpindexerColorSensors intakeColor;
    public final IntakeDistance intakeDistance;
    public final TableCompartmentManager tableCompartments;
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
        Globals.isTeleOp = pathSupplier == null;
        //octocanum = teleop ? new Octocanum(hardwareMap) : null;
        turret = new ServoTurret(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intakeMotor = new IntakeMotor(hardwareMap);
        table = new Table(hardwareMap, Encoder.fromMotor(drivetrain.leftRear));
        popper = new Popper(hardwareMap);
        hood = new Hood(hardwareMap);
        intakeColor = new SpindexerColorSensors(hardwareMap);
        //intakeDistance = new IntakeDistance(hardwareMap);
        intakeDistance = null;
        INSTANCE = this;
        RobotStateHandler.CycleState.DRIVE_TO_SHOOT.init(drivetrain.follower, turret, hood, flywheel, Globals.isTeleOp);
        tableCompartments = new TableCompartmentManager(intakeColor, table::getState);
        if (!Globals.isTeleOp) initialize();
    }

    public void initialize() {
        popper.setPosition(Popper.NEUTRAL);
        table.setPosition(Table.BALL1);
        turret.move(ServoTurretState.PresetState.REST);
    }

    public void update() {
        clearBulkCache();
        drivetrain.update();
        intakeColor.update();
        flywheel.update();
        turret.update();
        intakeMotor.update();
        table.update();
        popper.update();
        //intakeDistance.update();
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
        return Globals.isTeleOp;
    }

    public CycleState getRobotState() {
        return robotState;
    }

    public void setRobotState(Message message) {
        if (message instanceof DriveMessage driveState)
            CycleState.DRIVE_TO_SHOOT.INSTANCE = driveState.driveState();
        else if (message instanceof IntakeMessage intakeMessage)
            CycleState.INTAKE.INSTANCE = intakeMessage.intakeState();
        else if (message instanceof CycleState.Intake)
            CycleState.INTAKE.INSTANCE = RobotStateHandler.IntakeState.INTAKING;
        robotState = message.cycleState();
    }

    public ICommand sort() {
        return new Lazy(() -> {
            if (Globals.randomizationState != null) return table.setState(tableCompartments::sort);
            return Commands.NOOP;
        });
    }

    public ICommand sortAuto() {
        return new Sequential(
                new Instant(() -> table.setStateCommandless(Table.RelativeState.values()[tableCompartments.sort()])),
                new Wait(400)
        );
    }

    public ICommand shootAll() {
        return new Sequential(
                new WaitUntil(() -> drivetrain.canShoot),
                new Instant(intakeMotor::intake),
                table.shoot()
        );
    }

    public ICommand shootAll(Supplier<Double> delay) {
        AtomicReference<float[]> shootSequence = new AtomicReference<>();
        return new Conditional(
                () -> delay.get() < 10,
                shootAll(),
                new Sequential(
                        new WaitUntil(() -> drivetrain.canShoot),
                        new Instant(() -> {
                            intakeMotor.intakeSlow();
                            shootSequence.set(table.getState().getShootStates());
                        }),
                        table.setPos(() -> shootSequence.get()[0]),
                        new Wait(delay.get()),
                        table.setPos(() -> shootSequence.get()[1]),
                        new Wait(delay.get()),
                        table.setPos(() -> shootSequence.get()[2] + Table.FULL_REVOLUTION / 3),
                        new Instant(tableCompartments::removeAll)
                )
        );
    }

    public ICommand autoFastShoot() {
        return new Sequential(
                new Instant(intakeMotor::intake),
                new Lazy(() -> {
                    float pos = switch (table.getState()) {
                        case BALL0 -> Table.BALL0_END;
                        case BALL1 -> Table.BALL1_END;
                        case BALL2 -> Table.BALL2_END;
                    };

                    return new Sequential(
                            new Instant(() -> table.setPosition(pos)),
                            new Wait(650)
                    );
                })
        );
    }

    public ICommand autoShoot(Supplier<Double> delay) {
        AtomicReference<float[]> shootSequence = new AtomicReference<>();
        return new Conditional(
                () -> delay.get() < 10,
                autoFastShoot(),
                new Sequential(
                        new Instant(() -> {
                            intakeMotor.intake();
                            shootSequence.set(table.getState().getShootStates());
                        }),
                        new Instant(() -> table.setPosition(shootSequence.get()[0])),
                        new Wait(delay.get() + 600),
                        new Instant(() -> table.setPosition(shootSequence.get()[1])),
                        new Wait(delay.get() + 600),
                        new Instant(() -> table.setPosition(shootSequence.get()[2] + Table.FULL_REVOLUTION / 3)),
                        new Wait(500),
                        new Instant(tableCompartments::removeAll)
                )
        );
    }

    public ICommand shootAll(double delay) {
        return shootAll(() -> delay);
    }

    public ICommand resetShooter() {
        return new Instant(() -> {
            flywheel.stop();
            hood.rest();
        });
    }

    public ICommand resetAfterShooting() {
        return new Parallel(
                new Instant(drivetrain::stopHoldPose),
                resetShooter(),
                resetTableTeleOp()
        );
    }

    public ICommand resetTable() {
        return new Sequential(
                new Instant(intakeMotor::stop),
                popper.neutral(),
                table.reset(),
                new Instant(intakeMotor::intake)
        );
    }

    public ICommand resetTableTeleOp() {
        return new Sequential(
                new Instant(intakeMotor::stop),
                table.reset(),
                new Instant(intakeMotor::intake),
                popper.neutral()
        );
    }

    public ICommand sortAndShootAuto() {
        return new Sequential(
                sortAuto(),
                popper.pop(),
                autoShoot(() -> Globals.randomizationState == null ? 0.0 : Table.SLOW_SHOOT_DELAY)
        );
    }

    public Notifier shootAndReset() {
        return new Notifier(c -> new Sequential(
                shootAll(),
                Channels.send(c, Channels::signal),
                new Parallel(
                        new Sequential(
                                resetTable(),
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

    public void shootCorner() {
        hood.corner();
        flywheel.corner();
    }

    public ICommand shootFar() {
        return new Sequential(
                new Instant(() -> {
                    hood.far();
                    flywheel.far();
                }),
                new Instant(() -> turret.setPosition(Globals.allianceColor == AllianceColor.Red ? ServoTurret.FAR_PRESET_RED : ServoTurret.FAR_PRESET_BLUE))
        );
    }

    public void shootMedium() {
        hood.medium();
        flywheel.medium();
    }
}
