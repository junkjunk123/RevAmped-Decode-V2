package org.firstinspires.ftc.teamcode.revamped;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.revamped.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.ColorManager;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.IntakeDistance;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.IntakeThread;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.TableCompartmentManager;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.revamped.utils.PathSupplier;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.Encoder;

import java.util.List;

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
    private RobotState robotState;

    public interface Message {
        RobotState robotState();

        class DriveMessage implements Message {
            public final DriveState driveState;

            public DriveMessage(DriveState driveState) {
                this.driveState = driveState;
            }

            @Override
            public RobotState robotState() {
                return CycleState.DRIVE_TO_SHOOT;
            }
        }

        class IntakeMessage implements Message {
            public final IntakeState intakeState;

            public IntakeMessage(IntakeState intakeState) {
                this.intakeState = intakeState;
            }

            @Override
            public RobotState robotState() {
                return CycleState.INTAKE;
            }
        }
    }

    public interface RobotState {
        void update();
    }

    public interface CycleState extends RobotState {
        DriveToShoot DRIVE_TO_SHOOT = new DriveToShoot();
        Intake INTAKE = new Intake();
        Shoot SHOOT = new Shoot();

        class Intake implements CycleState {
            IntakeState INSTANCE = IntakeState.INTAKING;
            IntakeThread intakeThread = new IntakeThread();

            @Override
            public void update() {
                if (INSTANCE == IntakeState.INTAKING) {}
            }
        }

        class DriveToShoot implements CycleState {
            DriveState INSTANCE = DriveState.PASSIVE;
            TrackingThread autoTracker;

            public void init(Follower follower, Turret turret, Hood hood, Flywheel flywheel, boolean isTeleOp) {
                autoTracker = new TrackingThread(follower, turret, flywheel, hood, isTeleOp);
            }

            @Override
            public void update() {
                if (INSTANCE == DriveState.AUTO_TRACKING)
                    autoTracker.update();
            }
        }

        class Shoot implements CycleState {
            @Override
            public void update() {}
        }
    }

    public enum IntakeState {
        INTAKING,
        SORTING;
    }

    public enum DriveState {
        AUTO_TRACKING,
        PASSIVE
    }


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
        CycleState.DRIVE_TO_SHOOT.init(drivetrain.follower, turret, hood, flywheel, teleop);
        tableCompartments = new TableCompartmentManager();
        if (!teleop) init();
    }

    public void init() {
        hood.rest();
        popper.neutral();
        table.reset();
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

    public RobotState getRobotState() {
        return robotState;
    }

    public void setRobotState(Message message) {
        if (message instanceof Message.DriveMessage driveState)
            CycleState.DRIVE_TO_SHOOT.INSTANCE = driveState.driveState;
        else if (message instanceof Message.IntakeMessage intakeMessage)
            CycleState.INTAKE.INSTANCE = intakeMessage.intakeState;
        robotState = message.robotState();
    }

    public void sort() {
        table.setState(tableCompartments.sort(table.getState().ordinal()));
    }

    public ICommand shootAll() {
        return new Sequential(
                new Instant(() -> {
                    intakeMotor.intake();
                    table.fullRotation();
                }),
                new WaitUntil(table::reached)
        );
    }

    public ICommand shootAll(double delay) {
        float[] shootSequence = table.getState().getShootStates();
        assert shootSequence != null && shootSequence.length > 2;
        return new Sequential(
                new Instant(() -> {
                    intakeMotor.intakeSlow();
                    table.setPosition(shootSequence[0]);
                }),
                new WaitUntil(table::reached),
                new Instant(intakeMotor::stop),
                new Wait(delay),
                new Instant(() -> {
                    table.setPosition(shootSequence[1]);
                    intakeMotor.intakeSlow();
                }),
                new WaitUntil(table::reached),
                new Instant(intakeMotor::shooting),
                new Wait(delay),
                new Instant(() -> table.setPosition(shootSequence[2] + Table.FULL_REVOLUTION / 3)),
                new WaitUntil(table::reached)
        );
    }
}
