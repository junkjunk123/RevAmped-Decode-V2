package org.firstinspires.ftc.teamcode.revamped;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.revamped.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.ColorManager;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.IntakeDistance;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.IntakeThread;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.revamped.utils.Continuations.SimpleContinuation;
import org.firstinspires.ftc.teamcode.revamped.utils.PathSupplier;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.Encoder;

import java.util.List;

import dev.frozenmilk.dairy.mercurial.continuations.Actors;
import dev.frozenmilk.dairy.mercurial.continuations.Continuation;

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
    private boolean teleop = false;
    private final List<LynxModule> hubs;
    private final Actors.Actor<RobotState, Message> actor;

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

    public interface RobotState extends SimpleContinuation { }

    public interface CycleState extends RobotState {
        DriveToShoot DRIVE_TO_SHOOT = new DriveToShoot();
        Intake INTAKE = new Intake();
        Shoot SHOOT = new Shoot();

        class Intake implements CycleState {
            IntakeState INSTANCE = IntakeState.INTAKING;
            IntakeThread intakeThread = new IntakeThread();

            @NonNull
            @Override
            public Continuation apply() {
                return null;
            }
        }

        class DriveToShoot implements CycleState {
            static DriveState INSTANCE = DriveState.PASSIVE;
            static TrackingThread autoTracker;

            @Override
            public Continuation apply() {
                if (INSTANCE == DriveState.AUTO_TRACKING)
                    autoTracker.apply();
                return this;
            }

            public void init(Follower follower, Turret turret, Hood hood, Flywheel flywheel, boolean isTeleOp) {
                autoTracker = new TrackingThread(follower, turret, flywheel, hood, isTeleOp);
            }
        }

        class Shoot implements CycleState {
            @NonNull
            @Override
            public Continuation apply() {
                return Continuation.halt();
            }
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
        actor = null;
    }

    public void init() {
        hood.rest();
        popper.neutral();
        table.reset();
    }

    public Continuation update = new Continuation() {
        @NonNull
        @Override
        public Continuation apply() {
            clearBulkCache();
            intakeColor.update();
            flywheel.update();
            turret.update();
            intakeMotor.update();
            table.update();
            popper.update();
            intakeDistance.update();
            hood.update();
            return update;
        }

        @Nullable
        @Override
        public StackTraceElement[] getStackTrace() {
            return null;
        }
    };

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
}
