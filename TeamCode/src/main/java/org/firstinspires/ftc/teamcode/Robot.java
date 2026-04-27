package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Parallel;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.CycleState;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.DriveMessage;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.IntakeMessage;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler.Message;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeArtifactDetector;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeGate;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeTilt;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Splitter;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.intake.TableCompartmentManager;
import org.firstinspires.ftc.teamcode.mechanisms.lift.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.FeederWheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.GyroThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretState;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerColorSensors;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeBlobCamera;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeLimelight;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channels;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Notifier;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.hardware.HwVoltageSensor;
import org.firstinspires.ftc.teamcode.utils.math.Z3Element;
import org.firstinspires.ftc.teamcode.utils.math.projectile.TrackState;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
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
    public final IntakeArtifactDetector intakeDistance;
    public final IntakeArtifactDetector frontDistance;
    public final FeederWheel feederWheel;
    public final IntakeTilt intakeTilt;
    public final IntakeGate intakeGate;
    public final Splitter splitter;
    public final TableCompartmentManager tableCompartments;
    public final DecodeLimelight limelight;
    public final Lift lift;
    public final DecodeBlobCamera intakeCamera;
    public final HwVoltageSensor voltageSensor;

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
        voltageSensor = new HwVoltageSensor(hardwareMap);
        drivetrain = pathSupplier != null ? new Drivetrain(hardwareMap, pathSupplier) : new Drivetrain(hardwareMap);
        Globals.isTeleOp = pathSupplier == null;
        turret = new ServoTurret(hardwareMap);
        flywheel = new Flywheel(hardwareMap, voltageSensor);
        intakeMotor = new IntakeMotor(hardwareMap);
        table = new Table(hardwareMap, Encoder.fromMotor(drivetrain.leftFront));
        popper = new Popper(hardwareMap);
        hood = new Hood(hardwareMap, voltageSensor);
        intakeColor = new SpindexerColorSensors(hardwareMap);
        intakeDistance = new IntakeArtifactDetector(hardwareMap, "intakeDistance", 25);
        frontDistance = new IntakeArtifactDetector(hardwareMap, "frontDistance", 100, 0);
        feederWheel = new FeederWheel(hardwareMap);
        intakeTilt = new IntakeTilt(hardwareMap);
        intakeGate = new IntakeGate(hardwareMap);
        splitter = new Splitter(hardwareMap);
        limelight = new DecodeLimelight(hardwareMap);
        lift = new Lift(hardwareMap);
        intakeCamera = new DecodeBlobCamera(hardwareMap);
        INSTANCE = this;
        RobotStateHandler.CycleState.DRIVE_TO_SHOOT.init(drivetrain.follower, turret, hood, flywheel, Globals.isTeleOp);
        tableCompartments = new TableCompartmentManager(intakeColor, intakeDistance, frontDistance, table::getState);
        if (!Globals.isTeleOp) initialize();
    }

    public void initialize() {
        if (!Globals.isTeleOp) {
            table.setPosition(Table.BALL1);
            popper.neutralCommandless();
        }

        turret.move(ServoTurretState.PresetState.REST);
        intakeTilt.intake();
        intakeGate.setClose();
        splitter.setPositionNeutral();
    }

    public void update() {
        clearBulkCache();
        voltageSensor.update();
        drivetrain.update();
        flywheel.update();
        feederWheel.update();
        limelight.update();
        intakeCamera.update();
        intakeColor.update();
        turret.update();
        intakeMotor.update();
        table.update();
        popper.update();
        intakeDistance.update();
        intakeTilt.update();
        intakeGate.update();
        splitter.update();
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
            AtomicInteger indexLeft = new AtomicInteger();
            AtomicInteger currentIndex = new AtomicInteger();
            AtomicInteger indexRight = new AtomicInteger();
            if (Globals.randomizationState != null)
                return new Sequential(
                        new Instant(() -> {
                            currentIndex.set(table.getState().ordinal());
                            Z3Element index = new Z3Element(currentIndex.get());
                            indexLeft.set(index.plus(1).getVal());
                            indexRight.set(index.minus(1).getVal());
                            tableCompartments.compartmentColors[indexLeft.get()] = intakeColor.leftColorSensor.getColor();
                            tableCompartments.compartmentColors[indexRight.get()] = intakeColor.rightColorSensor.getColor();
                        }),
                        new Lazy(() -> {
                            List<ArtifactColor> colors = List.of(tableCompartments.compartmentColors[indexLeft.get()],
                                    tableCompartments.compartmentColors[indexRight.get()]);
                            if (colors.contains(ArtifactColor.NONE)) return Commands.NOOP;
                            if (colors.contains(ArtifactColor.GREEN)) {
                                return new Sequential(
                                        new Instant(() -> tableCompartments.compartmentColors[currentIndex.get()] = ArtifactColor.PURPLE),
                                        table.setState(() -> new Z3Element(currentIndex.get()).plus(tableCompartments.sort()).getVal())
                                );
                            }
                            else {
                                return new Sequential(
                                        new Instant(() -> tableCompartments.compartmentColors[currentIndex.get()] = ArtifactColor.GREEN),
                                        table.setState(() -> new Z3Element(currentIndex.get()).plus(tableCompartments.sort()).getVal())
                                );
                            }
                        })
                );
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
                new Instant(() -> {
                    intakeTilt.intake();
                    intakeMotor.stop();
                    CycleState.INTAKE.update = true;
                }),
                new Conditional(
                    () -> hood.atState(Hood.HoodState.FAR),
                    new Parallel(
                        new Sequential(
                            new Wait(50),
                            new Instant(hood::farHoodComp)
                            ),
                        table.shoot()
                    ),
                    table.shoot()
                )
        );
    }

    public ICommand shootAll(Supplier<Double> delay) {
        AtomicReference<float[]> shootSequence = new AtomicReference<>();
        return new Conditional(
                () -> delay.get() < 10,
                shootAll(),
                new Sequential(
                        new Instant(() -> {
                            intakeTilt.intake();
                            intakeMotor.intake();
                            CycleState.INTAKE.update = true;
                        }),
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

    public ICommand autoFastShoot(IntakeTilt.TiltState tiltState) {
        return new Sequential(
                new Instant(() -> {
                    if (tiltState.equals(IntakeTilt.TiltState.INTAKE)) intakeTilt.intake();
                    else if (tiltState.equals(IntakeTilt.TiltState.GATE_INTAKE)) intakeTilt.gateIntake();
                    intakeGate.open();
                }),
                new Conditional(
                        () -> hood.atState(Hood.HoodState.FAR),
                        new Parallel(
                                new Sequential(
                                        new Wait(50),
                                        new Instant(hood::farHoodComp)
                                ),
                                new Lazy(() -> {
                                    float pos = switch (table.getState()) {
                                        case BALL0 -> Table.BALL0_END;
                                        case BALL1 -> Table.BALL1_END;
                                        case BALL2 -> Table.BALL2_END;
                                    };

                                    return new Sequential(
                                            new Instant(() -> table.setPosition(pos)),
                                            new Wait(500)
                                    );
                                })
                        ),
                        new Lazy(() -> {
                            float pos = switch (table.getState()) {
                                case BALL0 -> Table.BALL0_END;
                                case BALL1 -> Table.BALL1_END;
                                case BALL2 -> Table.BALL2_END;
                            };

                            return new Sequential(
                                    new Instant(() -> table.setPosition(pos)),
                                    new Wait(500)
                            );
                        })
                ),
                new Instant(tableCompartments::removeAll)
        );
    }

    public ICommand autoFastShoot() {
        return autoFastShoot(IntakeTilt.TiltState.INTAKE);
    }

    public ICommand autoShoot(Supplier<Double> delay) {
        AtomicReference<float[]> shootSequence = new AtomicReference<>();
        return new Conditional(
                () -> delay.get() < 10,
                autoFastShoot(),
                new Sequential(
                        new Instant(() -> {
                            intakeMotor.intake();
                            intakeTilt.intake();
                            intakeGate.open();
                            shootSequence.set(table.getState().getShootStates());
                        }),
                        new Instant(() -> table.setPosition(shootSequence.get()[0])),
                        new Wait(delay.get() + 300),
                        new Instant(() -> table.setPosition(shootSequence.get()[1])),
                        new Wait(delay.get() + 300),
                        new Instant(() -> table.setPosition(shootSequence.get()[2] + Table.FULL_REVOLUTION / 3)),
                        new Wait(200),
                        new Instant(tableCompartments::removeAll),
                        new Instant(tableCompartments.intakeThread::reset)
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
            feederWheel.stop();
        });
    }

    public ICommand resetAfterShooting() {
        return new Parallel(
                new Instant(drivetrain::stopHoldPose),
                resetShooter(),
                resetTableTeleOp()
        );
    }

    public ICommand resetTableTeleOp() {
        return new Sequential(
                new Parallel(
                    new Instant(intakeMotor::stop),
                    intakeGate.open(),
                    table.reset(),
                    new Sequential(
                            new Wait(400),
                            new Instant(() -> {
                                intakeMotor.intake();
                                feederWheel.intakeState();
                            })
                    )
                ),
                new Parallel(
                        popper.block(),
                        splitter.activate()
                )
        );
    }

    public void shootNear() {
        hood.near();
        flywheel.near();
    }

    public void shootCorner() {
        hood.corner();
        flywheel.corner();
    }

    public void shootFar() {
        flywheel.far();
        hood.far();
    }

    public void shootMedium() {
        hood.medium();
        flywheel.medium();
    }

    public void close() {
        limelight.close();
        intakeCamera.close();
    }

    public ICommand lift() {
        return new Sequential(
                new Instant(() -> {
                    limelight.close();
                    turret.deenergize();
                    popper.deenergize();
                    intakeMotor.deenergize();
                    table.deenergize();
                    hood.deenergize();
                    intakeColor.close();
                    frontDistance.close();
                    feederWheel.deenergize();
                    intakeTilt.deenergize();
                    intakeGate.deenergize();
                    flywheel.deenergize();
                    splitter.deenergize();
                }),
                lift.lift()
        );
    }

    public ICommand intake() {
        return intake(0);
    }

    public ICommand intake(float tiltFinetune) {
        return new Sequential(
                new Instant(() -> intakeTilt.intake(tiltFinetune)),
                intakeGate.open(),
                new Instant(() -> {
                    intakeMotor.intake();
                    feederWheel.intakeState();
                }),
                new Wait(300),
                splitter.activate()
        );
    }

    public ICommand gateIntake() {
        return new Sequential(
                new Instant(intakeTilt::gateIntake),
                intakeGate.open(),
                new Instant(() -> {
                    intakeMotor.intake();
                    feederWheel.intakeState();
                }),
                new Wait(300),
                splitter.activate()
        );
    }
}
