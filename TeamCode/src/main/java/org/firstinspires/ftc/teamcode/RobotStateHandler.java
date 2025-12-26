package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.mechanisms.TeleOpStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;

import java.util.HashMap;

public class RobotStateHandler {
    public interface Message {
        CycleState cycleState();
    }

    public static class DriveMessage implements Message {
        public static DriveMessage PASSIVE = new DriveMessage(DriveState.PASSIVE);
        public static DriveMessage AUTO_TRACKING = new DriveMessage(DriveState.AUTO_TRACKING);

        public final DriveState driveState;

        public DriveMessage(DriveState driveState) {
            this.driveState = driveState;
        }

        @Override
        public CycleState cycleState() {
            return CycleState.DRIVE_TO_SHOOT;
        }
    }

    public static class IntakeMessage implements Message {
        public static IntakeMessage INTAKING = new IntakeMessage(IntakeState.INTAKING);
        public static IntakeMessage SORTING = new IntakeMessage(IntakeState.SORTING);

        public final IntakeState intakeState;

        public IntakeMessage(IntakeState intakeState) {
            this.intakeState = intakeState;
        }

        @Override
        public CycleState cycleState() {
            return CycleState.INTAKE;
        }
    }

    public interface CycleState extends TeleOpStateHandler.GraphElement, Message {
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
            public IntakeThread intakeThread = new IntakeThread();

            @Override
            public void update() {
                if (INSTANCE == IntakeState.INTAKING)
                    intakeThread.update();
            }

            public double[] getTransitionVector() {
                return new double[] {1,1,0};
            }
        }

        class DriveToShoot implements CycleState {
            public DriveState INSTANCE = DriveState.AUTO_TRACKING;
            public TrackingThread autoTracker;

            public void init(Follower follower, Turret turret, Hood hood, Flywheel flywheel, boolean isTeleOp) {
                autoTracker = new TrackingThread(follower, turret, flywheel, hood, isTeleOp);
            }

            @Override
            public void update() {
                if (INSTANCE == DriveState.AUTO_TRACKING)
                    autoTracker.update();
            }

            public double[] getTransitionVector() {
                return new double[] {1,1,1};
            }
        }

        class Shoot implements CycleState {
            @Override
            public void update() {}

            public double[] getTransitionVector() {
                return new double[] {1,0,1};
            }
        }

        default CycleState getCurrentState() {
            return this;
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

    public static TeleOpStateHandler createTeleOpStateHandler(Robot robot) {
        HashMap<CycleState, Integer> stateMap = new HashMap<>();
        stateMap.put(RobotStateHandler.CycleState.INTAKE, 0);
        stateMap.put(RobotStateHandler.CycleState.DRIVE_TO_SHOOT, 1);
        stateMap.put(RobotStateHandler.CycleState.SHOOT, 2);
        return new TeleOpStateHandler(CycleState.INTAKE, stateMap, robot::setRobotState);
    }
}
