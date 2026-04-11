package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.ColorManager;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeArtifactDetector;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistance;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerColorSensors;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.utils.ArtifactColor;

import java.util.List;

public class RobotStateHandler {
    public interface Message {
        CycleState cycleState();
    }

    public record DriveMessage(DriveState driveState) implements Message {
        public static DriveMessage PASSIVE = new DriveMessage(DriveState.PASSIVE);
        public static DriveMessage AUTO_TRACKING = new DriveMessage(DriveState.AUTO_TRACKING);

        @Override
        public CycleState cycleState() {
                return CycleState.DRIVE_TO_SHOOT;
            }
    }

    public record IntakeMessage(IntakeState intakeState) implements Message {
        public static IntakeMessage INTAKING = new IntakeMessage(IntakeState.INTAKING);
        public static IntakeMessage SORTING = new IntakeMessage(IntakeState.SORTING);

        @Override
        public CycleState cycleState() {
                return CycleState.INTAKE;
            }
    }

    public non-sealed interface CycleState extends TeleOpStateHandler.GraphElement, Message {
        DriveToShoot DRIVE_TO_SHOOT = new DriveToShoot();
        Intake INTAKE = new Intake();
        Shoot SHOOT = new Shoot();

        void update();

        double[] getTransitionVector();

        @Override
        default CycleState cycleState() {
            return this;
        }

        class Intake implements CycleState {
            public IntakeState INSTANCE = IntakeState.INTAKING;
            public boolean update;
            public IntakeThread intakeThread;

            @Override
            public void update() {
                if (update && INSTANCE == IntakeState.INTAKING)
                    intakeThread.update();
            }

            public void init(SpindexerColorSensors colorManager, ArtifactColor[] tableCompartments, IntakeArtifactDetector intakeDistance, IntakeArtifactDetector frontSensor) {
                INSTANCE = IntakeState.INTAKING;
                intakeThread = new IntakeThread(tableCompartments, colorManager, intakeDistance, frontSensor);
            }

            public double[] getTransitionVector() {
                return new double[] {1,1,0};
            }

            @NonNull
            @Override
            public String toString() {
                return "intake: " + INSTANCE.name();
            }
        }

        class DriveToShoot implements CycleState {
            public DriveState INSTANCE = DriveState.AUTO_TRACKING;
            public TrackingThread autoTracker;

            public void init(Follower follower, ServoTurret turret, Hood hood, Flywheel flywheel, boolean isTeleOp) {
                INSTANCE = DriveState.AUTO_TRACKING;
                autoTracker = new TrackingThread(follower, turret, flywheel, hood);
            }

            @Override
            public void update() {
                //if (INSTANCE == DriveState.AUTO_TRACKING)
                    //autoTracker.update();
            }

            public static void toggleDefault() {
                if (CycleState.DRIVE_TO_SHOOT.INSTANCE == DriveState.AUTO_TRACKING) {
                    CycleState.DRIVE_TO_SHOOT.INSTANCE = DriveState.PASSIVE;
                } else {
                    CycleState.DRIVE_TO_SHOOT.INSTANCE = DriveState.AUTO_TRACKING;
                }
            }

            public double[] getTransitionVector() {
                return new double[] {1,1,1};
            }

            @NonNull
            @Override
            public String toString() {
                return "driveToShoot: " + INSTANCE.name();
            }
        }

        class Shoot implements CycleState {
            @Override
            public void update() {}

            public double[] getTransitionVector() {
                return new double[] {1,0,1};
            }

            @NonNull
            @Override
            public String toString() {
                return "shoot";
            }
        }

        default CycleState getCurrentState() {
            return this;
        }
    }

    public enum IntakeState {
        INTAKING,
        SORTING
    }

    public enum DriveState {
        AUTO_TRACKING,
        PASSIVE
    }

    public static TeleOpStateHandler createTeleOpStateHandler(Robot robot) {
        List<CycleState> stateMap = List.of(CycleState.INTAKE, CycleState.DRIVE_TO_SHOOT, CycleState.SHOOT);
        return new TeleOpStateHandler(CycleState.INTAKE, stateMap, c -> {
            if (c.equals(CycleState.INTAKE)) robot.setRobotState(IntakeMessage.INTAKING);
            else robot.setRobotState(c);
        });
    }
}
