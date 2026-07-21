package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.math.MathFunctions.quadraticFit;
import static org.firstinspires.ftc.teamcode.pedro.Tuning.changes;
import static org.firstinspires.ftc.teamcode.pedro.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedro.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedro.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedro.Tuning.stopRobot;
import static org.firstinspires.ftc.teamcode.pedro.Tuning.telemetryM;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.NanoTimer;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.control.SquIDBrakingController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Stack;


/**
 * This is the Tuning class. It contains a selection menu for various tuning OpModes.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 6/26/2025
 */
@Configurable
@TeleOp(name = "Tuning", group = "Pedro Pathing")
public class Tuning extends SelectableOpMode {
    public static Follower follower;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    public Tuning() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Localization", l -> {
                l.add("Localization Test", LocalizationTest::new);
                l.add("Forward Tuner", ForwardTuner::new);
                l.add("Lateral Tuner", LateralTuner::new);
                l.add("Turn Tuner", TurnTuner::new);
                l.add("Offsets Tuner", OffsetsTuner::new);
                l.add("Automated Offsets Tuner", AutomatedOffsetsTuner::new);
            });
            s.folder("Automatic", a -> {
                a.add("Forward Velocity Tuner", ForwardVelocityTuner::new);
                a.add("Lateral Velocity Tuner", LateralVelocityTuner::new);
                a.add("Forward Zero Power Acceleration Tuner", ForwardZeroPowerAccelerationTuner::new);
                a.add("Lateral Zero Power Acceleration Tuner", LateralZeroPowerAccelerationTuner::new);
                a.add("Heading Auto Tuner", HeadingAutoTuner::new);
                a.add("Translational Auto Tuner", TranslationalAutoTuner::new);
                a.add("Quadratic Damping Tuner", PredictiveBrakingTuner::new);
                a.add("TeleOp Braking Tuner", TeleOpBrakingTuner::new);
            });
            s.folder("Manual", p -> {
                p.add("Translational Tuner", TranslationalTuner::new);
                p.add("Heading Tuner", HeadingTuner::new);
                p.add("Drive Tuner", DriveTuner::new);
                p.add("Line Tuner", Line::new);
                p.add("Centripetal Tuner", CentripetalTuner::new);
            });
            s.folder("Tests", p -> {
                p.add("Line", Line::new);
                p.add("Triangle", Triangle::new);
                p.add("Circle", Circle::new);
            });
        });
    }

    @Override
    public void onSelect() {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());

        poseHistory = follower.getPoseHistory();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        Drawing.init();
    }

    @Override
    public void onLog(List<String> lines) {}

    public static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void draw() {
        Drawing.drawDebug(follower);
    }

    /** This creates a full stop of the robot by setting the drive motors to run at 0 power. */
    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }
}

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot.
 * You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
class LocalizationTest extends OpMode {
    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
    }

    /** This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.update(telemetry);

        draw();
    }
}

/**
 * This is the ForwardTuner OpMode. This tracks the forward movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the forward ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
class ForwardTuner extends OpMode {
    public static double DISTANCE = 48;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        follower.update();
        drawOnlyCurrent();
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("Pull your robot forward " + DISTANCE + " inches. Your forward ticks to inches will be shown on the telemetry.");
        telemetryM.update(telemetry);
        drawOnlyCurrent();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    @Override
    public void loop() {
        follower.update();

        telemetryM.debug("Distance Moved: " + (follower.getPose().getX() - 72));
        telemetryM.debug("The multiplier will display what your forward ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
        telemetryM.debug("Multiplier: " + (DISTANCE / ((follower.getPose().getX() - 72) / follower.getPoseTracker().getLocalizer().getForwardMultiplier())));
        telemetryM.update(telemetry);

        draw();
    }
}

/**
 * This is the LateralTuner OpMode. This tracks the strafe movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current distance in ticks to the specified distance in inches. So, to use this, run the
 * tuner, then pull/push the robot to the specified distance using a ruler on the ground. When you're
 * at the end of the distance, record the ticks to inches multiplier. Feel free to run multiple trials
 * and average the results. Then, input the multiplier into the strafe ticks to inches in your
 * localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 6/26/2025
 */
class LateralTuner extends OpMode {
    public static double DISTANCE = 48;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        follower.update();
        drawOnlyCurrent();
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("Pull your robot to the right " + DISTANCE + " inches. Your strafe ticks to inches will be shown on the telemetry.");
        telemetryM.update(telemetry);
        drawOnlyCurrent();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    @Override
    public void loop() {
        follower.update();

        telemetryM.debug("Distance Moved: " + (follower.getPose().getY() - 72));
        telemetryM.debug("The multiplier will display what your strafe ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
        telemetryM.debug("Multiplier: " + (DISTANCE / ((follower.getPose().getY() - 72) / follower.getPoseTracker().getLocalizer().getLateralMultiplier())));
        telemetryM.update(telemetry);

        draw();
    }
}

/**
 * This is the TurnTuner OpMode. This tracks the turning movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current angle in ticks to the specified angle in radians. So, to use this, run the
 * tuner, then pull/push the robot to the specified angle using a protractor or lines on the ground.
 * When you're at the end of the angle, record the ticks to inches multiplier. Feel free to run
 * multiple trials and average the results. Then, input the multiplier into the turning ticks to
 * radians in your localizer of choice.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 5/6/2024
 */
class TurnTuner extends OpMode {
    public static double ANGLE = 2 * Math.PI;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        follower.update();
        drawOnlyCurrent();
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("Turn your robot " + ANGLE + " radians. Your turn ticks to inches will be shown on the telemetry.");
        telemetryM.update(telemetry);

        drawOnlyCurrent();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated multiplier and draws the robot.
     */
    @Override
    public void loop() {
        follower.update();

        telemetryM.debug("Total Angle: " + follower.getTotalHeading());
        telemetryM.debug("The multiplier will display what your turn ticks to inches should be to scale your current angle to " + ANGLE + " radians.");
        telemetryM.debug("Multiplier: " + (ANGLE / (follower.getTotalHeading() / follower.getPoseTracker().getLocalizer().getTurningMultiplier())));
        telemetryM.update(telemetry);

        draw();
    }
}

/**
 * This is the OffsetsTuner OpMode. This tracks the movement of the robot as it turns 180 degrees,
 * and calculates what the robot's strafeX and forwardY offsets should be. Ensure that your strafeX and forwardY offsets
 * are set to 0 before running this OpMode. After running, input the displayed offsets into your localizer constants.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
class OffsetsTuner extends OpMode {
    @Override
    public void init() {
        follower.setStartingPose(new Pose(72,72));
        follower.update();
        drawOnlyCurrent();
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("Prerequisite: Make sure both your offsets are set to 0 in your localizer constants.");
        telemetryM.debug("Turn your robot " + Math.PI + " radians. Your offsets in inches will be shown on the telemetry.");
        telemetryM.update(telemetry);

        drawOnlyCurrent();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated offsets and draws the robot.
     */
    @Override
    public void loop() {
        follower.update();

        telemetryM.debug("Total Angle: " + follower.getTotalHeading());

        telemetryM.debug("The following values are the offsets in inches that should be applied to your localizer.");
        telemetryM.debug("strafeX: " + ((72.0-follower.getPose().getX()) / 2.0));
        telemetryM.debug("forwardY: " + ((72.0-follower.getPose().getY()) / 2.0));
        telemetryM.update(telemetry);

        draw();
    }
}

/**
 * This is the ForwardVelocityTuner autonomous follower OpMode. This runs the robot forwards at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with StrafeVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
class ForwardVelocityTuner extends OpMode {
    private final ArrayList<Double> velocities = new ArrayList<>();
    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;

    private boolean end;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the drive motors as well as the cache of velocities and the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches forward.");
        telemetryM.debug("Make sure you have enough room, since the robot has inertia after cutting power.");
        telemetryM.debug("After running the distance, the robot will cut power from the drivetrain and display the forward velocity.");
        telemetryM.debug("Press B on game pad 1 to stop.");
        telemetryM.debug("pose", follower.getPose());
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power. */
    @Override
    public void start() {
        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }
        follower.startTeleopDrive(true);
        follower.update();
        end = false;
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run forward enough, these last velocities recorded are
     * averaged and printed.
     */
    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }

        follower.update();
        draw();


        if (!end) {
            if (Math.abs(follower.getPose().getX()) > (DISTANCE + 72)) {
                end = true;
                stopRobot();
            } else {
                follower.setTeleOpDrive(1,0,0,true);
                //double currentVelocity = Math.abs(follower.getVelocity().getXComponent());
                double currentVelocity = Math.abs(follower.poseTracker.getLocalizer().getVelocity().getX());
                velocities.add(currentVelocity);
                velocities.remove(0);
            }
        } else {
            stopRobot();
            double average = 0;
            for (double velocity : velocities) {
                average += velocity;
            }
            average /= velocities.size();
            telemetryM.debug("Forward Velocity: " + average);
            telemetryM.debug("\n");
            telemetryM.debug("Press A to set the Forward Velocity temporarily (while robot remains on).");

            for (int i = 0; i < velocities.size(); i++) {
                telemetry.addData(String.valueOf(i), velocities.get(i));
            }

            telemetryM.update(telemetry);
            telemetry.update();

            if (gamepad1.aWasPressed()) {
                follower.setXVelocity(average);
                String message = "XMovement: " + average;
                changes.add(message);
            }
        }
    }
}

/**
 * This is the StrafeVelocityTuner autonomous follower OpMode. This runs the robot left at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with ForwardVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
class LateralVelocityTuner extends OpMode {
    private final ArrayList<Double> velocities = new ArrayList<>();

    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;

    private boolean end;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This initializes the drive motors as well as the cache of velocities and the Panels
     * telemetryM.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches to the left.");
        telemetryM.debug("Make sure you have enough room, since the robot has inertia after cutting power.");
        telemetryM.debug("After running the distance, the robot will cut power from the drivetrain and display the strafe velocity.");
        telemetryM.debug("Press B on Gamepad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    /** This starts the OpMode by setting the drive motors to run left at full power. */
    @Override
    public void start() {
        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }
        follower.startTeleopDrive(true);
        follower.update();
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run sideways enough, these last velocities recorded are
     * averaged and printed.
     */
    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }

        follower.update();
        draw();

        if (!end) {
            if (Math.abs(follower.getPose().getY()) > (DISTANCE + 72)) {
                end = true;
                stopRobot();
            } else {
                follower.setTeleOpDrive(0,1,0,true);
                double currentVelocity = Math.abs(follower.getVelocity().dot(new Vector(1, Math.PI / 2)));
                velocities.add(currentVelocity);
                velocities.remove(0);
            }
        } else {
            stopRobot();
            double average = 0;
            for (double velocity : velocities) {
                average += velocity;
            }
            average /= velocities.size();

            telemetryM.debug("Strafe Velocity: " + average);
            telemetryM.debug("\n");
            telemetryM.debug("Press A to set the Lateral Velocity temporarily (while robot remains on).");
            telemetryM.update(telemetry);

            if (gamepad1.aWasPressed()) {
                follower.setYVelocity(average);
                String message = "YMovement: " + average;
                changes.add(message);
            }
        }
    }
}

/**
 * This is the ForwardZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * forward until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
class ForwardZeroPowerAccelerationTuner extends OpMode {
    private final ArrayList<Double> accelerations = new ArrayList<>();
    public static double VELOCITY = 30;

    private double previousVelocity;
    private long previousTimeNano;

    private boolean stopping;
    private boolean end;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the drive motors as well as the Panels telemetryM. */
    @Override
    public void init_loop() {
        telemetryM.debug("The robot will run forward until it reaches " + VELOCITY + " inches per second.");
        telemetryM.debug("Then, it will cut power from the drivetrain and roll to a stop.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.debug("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.");
        telemetryM.debug("Press B on Gamepad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power. */
    @Override
    public void start() {
        follower.startTeleopDrive(false);
        follower.update();
        follower.setTeleOpDrive(1,0,0,true);
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }

        follower.update();
        draw();

        Vector heading = new Vector(1.0, follower.getPose().getHeading());
        if (!end) {
            if (!stopping) {
                if (follower.getVelocity().dot(heading) > VELOCITY) {
                    previousVelocity = follower.getVelocity().dot(heading);
                    previousTimeNano = System.nanoTime();
                    stopping = true;
                    follower.setTeleOpDrive(0,0,0,true);
                }
            } else {
                double currentVelocity = follower.getVelocity().dot(heading);
                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();
                if (currentVelocity < follower.getConstraints().getVelocityConstraint()) {
                    end = true;
                }
            }
        } else {
            double average = 0;
            for (double acceleration : accelerations) {
                average += acceleration;
            }
            average /= accelerations.size();

            telemetryM.debug("Forward Zero Power Acceleration (Deceleration): " + average);
            telemetryM.debug("\n");
            telemetryM.debug("Press A to set the Forward Zero Power Acceleration temporarily (while robot remains on).");
            telemetryM.update(telemetry);

            if (gamepad1.aWasPressed()) {
                follower.getConstants().setForwardZeroPowerAcceleration(average);
                String message = "Forward Zero Power Acceleration: " + average;
                changes.add(message);
            }
        }
    }
}

/**
 * This is the LateralZeroPowerAccelerationTuner autonomous follower OpMode. This runs the robot
 * to the left until a specified velocity is achieved. Then, the robot cuts power to the motors, setting
 * them to zero power. The deceleration, or negative acceleration, is then measured until the robot
 * stops. The accelerations across the entire time the robot is slowing down is then averaged and
 * that number is then printed. This is used to determine how the robot will decelerate in the
 * forward direction when power is cut, making the estimations used in the calculations for the
 * drive Vector more accurate and giving better braking at the end of Paths.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 3/13/2024
 */
class LateralZeroPowerAccelerationTuner extends OpMode {
    private final ArrayList<Double> accelerations = new ArrayList<>();
    public static double VELOCITY = 30;
    private double previousVelocity;
    private long previousTimeNano;
    private boolean stopping;
    private boolean end;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the drive motors as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetryM.debug("The robot will run to the left until it reaches " + VELOCITY + " inches per second.");
        telemetryM.debug("Then, it will cut power from the drivetrain and roll to a stop.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.debug("After stopping, the lateral zero power acceleration (natural deceleration) will be displayed.");
        telemetryM.debug("Press B on game pad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power. */
    @Override
    public void start() {
        follower.startTeleopDrive(false);
        follower.update();
        follower.setTeleOpDrive(0,1,0,true);
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
     * game pad 1 will stop the OpMode. When the robot hits the specified velocity, the robot will
     * record its deceleration / negative acceleration until it stops. Then, it will average all the
     * recorded deceleration / negative acceleration and print that value.
     */
    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }

        follower.update();
        draw();

        Vector heading = new Vector(1.0, follower.getPose().getHeading() - Math.PI / 2);
        if (!end) {
            if (!stopping) {
                if (Math.abs(follower.getVelocity().dot(heading)) > VELOCITY) {
                    previousVelocity = Math.abs(follower.getVelocity().dot(heading));
                    previousTimeNano = System.nanoTime();
                    stopping = true;
                    follower.setTeleOpDrive(0,0,0,true);
                }
            } else {
                double currentVelocity = Math.abs(follower.getVelocity().dot(heading));
                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();
                if (currentVelocity < follower.getConstraints().getVelocityConstraint()) {
                    end = true;
                }
            }
        } else {
            double average = 0;
            for (double acceleration : accelerations) {
                average += acceleration;
            }
            average /= accelerations.size();

            telemetryM.debug("Lateral Zero Power Acceleration (Deceleration): " + average);
            telemetryM.debug("\n");
            telemetryM.debug("Press A to set the Lateral Zero Power Acceleration temporarily (while robot remains on).");
            telemetryM.update(telemetry);

            if (gamepad1.aWasPressed()) {
                follower.getConstants().setLateralZeroPowerAcceleration(average);
                String message = "Lateral Zero Power Acceleration: " + average;
                changes.add(message);
            }
        }
    }
}

/**
 * This is the Translational PIDF Tuner OpMode. It will keep the robot in place.
 * The user should push the robot laterally to test the PIDF and adjust the PIDF values accordingly.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class TranslationalTuner extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate the translational PIDF(s)");
        telemetryM.debug("The robot will try to stay in place while you push it laterally.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's translational PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.deactivateAllPIDFs();
        follower.activateTranslational();
        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Push the robot laterally to test the Translational PIDF(s).");
        telemetryM.addData("Zero Line", 0);
        telemetryM.addData("Error X", follower.errorCalculator.getTranslationalError().getXComponent());
        telemetryM.addData("Error Y", follower.errorCalculator.getTranslationalError().getYComponent());
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Heading PIDF Tuner OpMode. It will keep the robot in place.
 * The user should try to turn the robot to test the PIDF and adjust the PIDF values accordingly.
 * It will try to keep the robot at a constant heading while the user tries to turn it.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class HeadingTuner extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate the heading PIDF(s).");
        telemetryM.debug("The robot will try to stay at a constant heading while you try to turn it.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's heading PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.deactivateAllPIDFs();
        follower.activateHeading();
        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Turn the robot manually to test the Heading PIDF(s).");
        telemetryM.addData("Zero Line", 0);
        telemetryM.addData("Error", follower.errorCalculator.getHeadingError());
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Drive PIDF Tuner OpMode. It will run the robot in a straight line going forward and back.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class DriveTuner extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private PathChain forwards;
    private PathChain backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the Panels telemetry.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("This will run the robot in a straight line going " + DISTANCE + "inches forward.");
        telemetryM.debug("The robot will go forward and backward continuously along the path.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.deactivateAllPIDFs();
        follower.activateDrive();

        forwards = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)))
                .setConstantHeadingInterpolation(0)
                .build();

        backwards = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)))
                .setConstantHeadingInterpolation(0)
                .build();

        follower.followPath(forwards);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Driving forward?: " + forward);
        telemetryM.addData("Zero Line", 0);
        telemetryM.addData("Error", follower.errorCalculator.getDriveErrors()[1]);
        telemetryM.update(telemetry);
    }
}

class HeadingAutoTuner extends OpMode {
    private static final double ALPHA_LARGE = 0.6;
    private static final double ALPHA_SMALL = 0.9;
    private static final double BETA = 1.0;
    private static final double POWER = 0.6;
    private static final double RUNTIME = 3;
    private static final int SAMPLES = 15;

    private double tau;
    private double lambda_small;
    private double lambda_large;
    private double K;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> angularVelocities = new ArrayList<>();
    private final NanoTimer timer = new NanoTimer();
    private boolean done = false;
    private double lastTime = 0.0;
    private double dt = 0.0;

    @Override
    public void init() {
    }

    @Override
    public void init_loop() {
        telemetryM.debug("This will turn continuously in place for " + RUNTIME + " seconds.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        timer.resetTimer();
        lastTime = timer.getElapsedTimeSeconds();
        follower.startTeleOpDrive(true);
        follower.setTeleOpDrive(0, 0, POWER, true);
        drawOnlyCurrent();
    }

    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            follower.setTeleOpDrive(0, 0, 0, true);
            requestOpModeStop();
        }

        double now = timer.getElapsedTimeSeconds();
        dt = now - lastTime;
        if (dt <= 0) dt = 1e-6;


        lastTime = now;

        follower.update();
        telemetryM.update(telemetry);
        draw();

        telemetryM.addData("done", done);
        telemetryM.addData("dt", String.format("%.6f s", dt));

        if (!done) {
            times.add(timer.getElapsedTimeSeconds());
            angularVelocities.add(Math.abs(follower.getAngularVelocity()));
            telemetryM.addData("angular velocity (rad/s)", String.format("%.4f", angularVelocities.get(angularVelocities.size() - 1)));

            if (timer.getElapsedTimeSeconds() >= RUNTIME) {
                done = true;
                systemIdentification();
                follower.setTeleOpDrive(0, 0, 0, true);
                telemetryM.addData("elapsed time (s)", String.format("%.4f", timer.getElapsedTimeSeconds()));
            } else {
                follower.setTeleOpDrive(0, 0, POWER, true);
                return;
            }
        }

        lambda_small = tau * ALPHA_SMALL;
        lambda_large = tau * ALPHA_LARGE;

        double kDLarge = getkD(lambda_large);
        double kPLarge = getkP(lambda_large);
        double kDSmall = getkD(lambda_small);
        double kPSmall = getkP(lambda_small);

        double feedforward = BETA / K;

        telemetryM.addData("Est tau (s)", String.format("%.4f", tau));
        telemetryM.addData("Est K (rad/s per power)", String.format("%.4f", K));
        telemetryM.addData("Lambda large (s)", String.format("%.4f", lambda_large));
        telemetryM.addData("Lambda small (s)", String.format("%.4f", lambda_small));
        telemetryM.addData("Large Coefficients", "kP=" + String.format("%.4f", kPLarge) + ", kD=" + String.format("%.4f", kDLarge));
        telemetryM.addData("Small Coefficients", "kP=" + String.format("%.4f", kPSmall) + ", kD=" + String.format("%.4f", kDSmall));
        telemetryM.addData("Heading Feedforward", "k=" + String.format("%.4f", feedforward));
    }

    private double getkP(double lambda) {
        return tau / (K * lambda * lambda);
    }

    private double getkD(double lambda) {
        return 1 / K * (2 * tau / lambda - 1);
    }

    private void systemIdentification() {
        int N = times.size();
        if (N < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, N - SAMPLES);
        double samples = N - start;
        double sum = 0;
        for (int i = start; i < N; i++) sum += angularVelocities.get(i);
        double A = sum / samples;
        this.K = A / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            double vel = angularVelocities.get(i) / POWER;
            if (vel > 0.8 * K) continue;
            if (vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        double[] linReg = linearFit(x.stream().toArray(Double[]::new), y.stream().toArray(Double[]::new));
        if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
        this.tau = -1.0/linReg[1];
    }

    public double[] linearFit(Double[] x, Double[] y) {
        int n = x.length;
        double sumX = 0, sumXY = 0, sumY = 0, sumX2 = 0;

        for (int i = 0; i < n; i++) {
            sumX += x[i];
            sumY += y[i];
            sumXY += x[i] * y[i];
            sumX2 += x[i] * x[i];
        }

        double m = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
        double b = (sumY - m * sumX) / n;
        return new double[] {b, m};
    }
}

@Config
class TranslationalAutoTuner extends OpMode {
    public static double kBrakeQuadratic = Constants.K_QUADRATIC_BRAKE;
    public static double kBrakeLinear = Constants.K_LINEAR_BRAKE;
    public static double BETA_LARGE = 0.6;
    public static double BETA_SMALL = 0.9;

    private static final double POWER = 0.4;
    private static final double RUNTIME = 1.2;
    private static final int SAMPLES = 15;

    private double tau;
    private double K;
    private double kV;
    private double kA;
    private double vMax = 0;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final NanoTimer timer = new NanoTimer();
    private boolean done = false;
    private double lastTime = 0.0;
    private double dt = 0.0;

    private Pose lastPose;
    private double totalDrift;

    @Override
    public void init() {
    }

    @Override
    public void init_loop() {
        telemetryM.debug("This will run continuously in place for " + RUNTIME + " seconds.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        timer.resetTimer();
        lastTime = timer.getElapsedTimeSeconds();
        follower.startTeleOpDrive(true);
        follower.setTeleOpDrive(POWER, 0, 0, true);
        drawOnlyCurrent();
        lastPose = follower.getPose();
    }

    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            follower.setTeleOpDrive(0, 0, 0, true);
            requestOpModeStop();
        }

        double now = timer.getElapsedTimeSeconds();
        dt = now - lastTime;
        if (dt <= 0) dt = 1e-6;


        lastTime = now;

        follower.update();
        telemetryM.update(telemetry);
        draw();

        telemetryM.addData("done", done);
        telemetryM.addData("dt", String.format("%.6f s", dt));

        if (!done) {
            times.add(timer.getElapsedTimeSeconds());

            double dy = follower.getPose().getY() - lastPose.getY();
            double forwardVelocity = Math.abs(follower.getVelocity().dot(new Vector(1, follower.getHeading())));
            double drift = dy / (forwardVelocity + 1e-6);
            totalDrift += drift;
            vMax = Math.max(vMax, forwardVelocity / POWER);

            velocities.add(forwardVelocity);
            telemetryM.addData("velocity (in/s)", String.format("%.4f", velocities.get(velocities.size() - 1)));

            if (timer.getElapsedTimeSeconds() >= RUNTIME) {
                done = true;
                systemIdentification();

                follower.setTeleOpDrive(0, 0, 0, true);
                telemetryM.addData("elapsed time (s)", String.format("%.4f", timer.getElapsedTimeSeconds()));
            } else {
                follower.setTeleOpDrive(POWER, 0, 0, true);
                return;
            }
        }

        double kP_large = calculatekP(BETA_LARGE);
        double kP_small = calculatekP(BETA_SMALL);
        double normalFeedforward = totalDrift / K;

        telemetryM.addData("Est tau (s)", String.format("%.4f", tau));
        telemetryM.addData("Est K (in/s per power)", String.format("%.4f", K));
        telemetryM.addData("Est kV", kV);
        telemetryM.addData("Est kA", kA);
        telemetryM.addData("Large Coefficients", "kP=" + String.format("%.4f", kP_large));
        telemetryM.addData("Small Coefficients", "kP=" + String.format("%.4f", kP_small));
        telemetryM.addData("Normal Feedforward", "k=" + String.format("%.4f", normalFeedforward));
    }

    private double calculatekP(double beta) {
        kV = 1 / K;
        kA = tau / K * beta;
        double denominator = kBrakeLinear + 2.0 * kBrakeQuadratic * vMax;
        double discriminant = kA - kV * denominator;

        if (discriminant < 0) return kV * kV / (4.0 * kA);
        double sqrt = (Math.sqrt(kA) - Math.sqrt(discriminant)) / denominator;
        return sqrt * sqrt;
    }

    private void systemIdentification() {
        int N = times.size();
        if (N < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, N - SAMPLES);
        double samples = N - start;
        double sum = 0;
        for (int i = start; i < N; i++) sum += velocities.get(i);
        double A = sum / samples;
        this.K = A / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            double vel = velocities.get(i) / POWER;
            if (vel > 0.8 * K) continue;
            if (vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        double[] linReg = linearFit(x.stream().toArray(Double[]::new), y.stream().toArray(Double[]::new));
        if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
        this.tau = -1.0/linReg[1];
    }

    public double[] linearFit(Double[] x, Double[] y) {
        int n = x.length;
        double sumX = 0, sumXY = 0, sumY = 0, sumX2 = 0;

        for (int i = 0; i < n; i++) {
            sumX += x[i];
            sumY += y[i];
            sumXY += x[i] * y[i];
            sumX2 += x[i] * x[i];
        }

        double m = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
        double b = (sumY - m * sumX) / n;
        return new double[] {b, m};
    }
}

/**
 * This is the Line Test Tuner OpMode. It will drive the robot forward and back
 * The user should push the robot laterally and angular to test out the drive, heading, and translational PIDFs.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class Line extends OpMode {
    public static double DISTANCE = 48;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    public static double FORWARD_P = 0.15;
    public static double BACKWARD_P = 0.15;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {

        telemetryM.debug("This will activate all the PIDF(s)");
        telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(DISTANCE + 72,72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (gamepad1.bWasPressed()) {
            BACKWARD_P += -0.05;
        }

        if (gamepad1.xWasPressed()) {
            BACKWARD_P += 0.05;
        }

        if (gamepad1.yWasPressed()) {
            FORWARD_P += 0.05;
        }

        if (gamepad1.aWasPressed()) {
            FORWARD_P -= 0.05;
        }

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.vectorCalculator.predictiveBrakingController.setCoefficients(
                        new PredictiveBrakingCoefficients(
                                BACKWARD_P,
                                0.090,
                                0.00125
                        )
                );
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.vectorCalculator.predictiveBrakingController = new SquIDBrakingController(
                        new PredictiveBrakingCoefficients(
                                FORWARD_P,
                                0.090,
                                0.00125
                        )
                );
                follower.followPath(forwards);
            }
        }

        FtcDashboard.getInstance().getTelemetry().addData("velocity", follower.getVelocity().getXComponent());
        FtcDashboard.getInstance().getTelemetry().update();

        telemetryM.debug("Driving Forward?: " + forward);
        telemetryM.debug("forward_P", FORWARD_P);
        telemetryM.debug("backward_P", BACKWARD_P);
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Centripetal Tuner OpMode. It runs the robot in a specified distance
 * forward and to the left. On reaching the end of the forward Path, the robot runs the backward
 * Path the same distance back to the start. Rinse and repeat! This is good for testing a variety
 * of Vectors, like the drive Vector, the translational Vector, the heading Vector, and the
 * centripetal Vector.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
class CentripetalTuner extends OpMode {
    public static double DISTANCE = 20;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This initializes the Follower and creates the forward and backward Paths.
     * Additionally, this initializes the Panels telemetry.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("This will run the robot in a curve going " + DISTANCE + " inches to the left and the same number of inches forward.");
        telemetryM.debug("The robot will go continuously along the path.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        forwards = new Path(new BezierCurve(new Pose(72,72), new Pose(Math.abs(DISTANCE) + 72,72), new Pose(Math.abs(DISTANCE) + 72,DISTANCE + 72)));
        backwards = new Path(new BezierCurve(new Pose(Math.abs(DISTANCE) + 72,DISTANCE + 72), new Pose(Math.abs(DISTANCE) + 72,72), new Pose(72,72)));

        backwards.setTangentHeadingInterpolation();
        backwards.reverseHeadingInterpolation();

        follower.followPath(forwards);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        follower.update();
        draw();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Driving away from the origin along the curve?: " + forward);
        telemetryM.update(telemetry);
    }
}

/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
class Triangle extends OpMode {

    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    private final Pose interPose = new Pose(24 + 72, -24 + 72, Math.toRadians(90));
    private final Pose endPose = new Pose(24 + 72, 24 + 72, Math.toRadians(45));

    private PathChain triangle;

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }
    }

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    @Override
    public void init_loop() {
        telemetryM.debug("This will run in a roughly triangular shape, starting on the bottom-middle point.");
        telemetryM.debug("So, make sure you have enough space to the left, front, and right to run the OpMode.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    /** Creates the PathChain for the "triangle".*/
    @Override
    public void start() {
        follower.setStartingPose(startPose);

        triangle = follower.pathBuilder()
                .addPath(new BezierLine(startPose, interPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
                .addPath(new BezierLine(interPose, endPose))
                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
                .addPath(new BezierLine(endPose, startPose))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();

        follower.followPath(triangle);
    }
}

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
class Circle extends OpMode {
    public static double RADIUS = 10;
    private PathChain circle;

    public void start() {
        circle = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(72, 72), new Pose(RADIUS + 72, 72), new Pose(RADIUS + 72, RADIUS + 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .addPath(new BezierCurve(new Pose(RADIUS + 72, RADIUS + 72), new Pose(RADIUS + 72, (2 * RADIUS) + 72), new Pose(72, (2 * RADIUS) + 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .addPath(new BezierCurve(new Pose(72, (2 * RADIUS) + 72), new Pose(-RADIUS + 72, (2 * RADIUS) + 72), new Pose(-RADIUS + 72, RADIUS + 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .addPath(new BezierCurve(new Pose(-RADIUS + 72, RADIUS + 72), new Pose(-RADIUS + 72, 72), new Pose(72, 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .build();
        follower.followPath(circle);
    }

    @Override
    public void init_loop() {
        telemetryM.debug("This will run in a roughly circular shape of radius " + RADIUS + ", starting on the right-most edge. ");
        telemetryM.debug("So, make sure you have enough space to the left, front, and back to run the OpMode.");
        telemetryM.debug("It will also continuously face the center of the circle to test your heading and centripetal correction.");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (follower.atParametricEnd()) {
            follower.followPath(circle);
        }
    }
}

class TeleOpBrakingTuner extends OpMode {
    private static final double[] TEST_POWERS =
            {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2};
    private static final double BRAKING_POWER = 0.0;

    private static final int DRIVE_TIME_MS = 1000;

    private enum State {
        START_MOVE,
        WAIT_DRIVE_TIME,
        APPLY_BRAKE,
        WAIT_BRAKE_TIME,
        RECORD,
        WAIT,
        DONE
    }

    private static class BrakeRecord {
        double timeMs;
        Pose pose;
        double velocity;

        BrakeRecord(double timeMs, Pose pose, double velocity) {
            this.timeMs = timeMs;
            this.pose = pose;
            this.velocity = velocity;
        }
    }

    private State state = State.START_MOVE;

    private final ElapsedTime timer = new ElapsedTime();

    private int iteration = 0;

    private Vector startPosition;
    private double measuredVelocity;

    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private final List<BrakeRecord> brakeData = new ArrayList<>();

    @Override
    public void init() {}

    @Override
    public void init_loop() {
        telemetryM.debug("The robot will move forwards and backwards starting at max speed and slowing down.");
        telemetryM.debug("Make sure you have enough room. Leave at least 4-5 feet.");
        telemetryM.debug("After stopping, kFriction and kBraking will be displayed.");
        telemetryM.debug("Make sure to turn the timer off.");
        telemetryM.debug("Press B on game pad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        Drawing.drawCurrent();
    }

    @Override
    public void start() {
        timer.reset();
        follower.update();
        follower.startTeleOpDrive(true);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        follower.update();

        if (gamepad1.b) {
            stopRobot();
            requestOpModeStop();
            return;
        }

        double direction = (iteration % 2 == 0) ? 1 : -1;

        switch (state) {
            case START_MOVE: {
                if (iteration >= TEST_POWERS.length) {
                    state = State.DONE;
                    break;
                }

                double currentPower = TEST_POWERS[iteration];
                follower.setMaxPower(currentPower);
                follower.setTeleOpDrive(direction, 0, 0, true);

                timer.reset();
                state = State.WAIT_DRIVE_TIME;
                break;
            }

            case WAIT_DRIVE_TIME: {
                if (timer.milliseconds() >= DRIVE_TIME_MS) {
                    measuredVelocity = follower.getVelocity().getMagnitude();
                    startPosition = follower.getPose().getAsVector();
                    state = State.APPLY_BRAKE;
                }
                break;
            }

            case APPLY_BRAKE: {
                follower.setTeleOpDrive(BRAKING_POWER * direction, 0, 0, true);

                timer.reset();
                state = State.WAIT_BRAKE_TIME;
                break;
            }

            case WAIT_BRAKE_TIME: {
                double t = timer.milliseconds();
                Pose currentPose = follower.getPose();
                double currentVelocity = follower.getVelocity().getMagnitude();

                brakeData.add(new BrakeRecord(t, currentPose, currentVelocity));

                if (follower.getVelocity().dot(new Vector(direction,
                        follower.getHeading())) <= 0) {
                    state = State.RECORD;
                }
                break;
            }

            case RECORD: {
                Vector endPosition = follower.getPose().getAsVector();
                double brakingDistance = endPosition.minus(startPosition).getMagnitude();

                velocityToBrakingDistance.add(new double[] {measuredVelocity, brakingDistance});

                telemetryM.debug("Test " + iteration,
                        String.format("v=%.3f  d=%.3f", measuredVelocity,
                                brakingDistance));
                telemetryM.update(telemetry);
                timer.reset();
                state = State.WAIT;
                break;
            }

            case WAIT: {
                if (timer.milliseconds() > 3000) {
                    iteration++;
                    state = State.START_MOVE;
                }

                break;
            }

            case DONE: {
                stopRobot();

                double[] coefficients = quadraticFit(velocityToBrakingDistance);

                telemetryM.debug("Tuning Complete");
                telemetryM.debug("Braking Profile:");
                telemetryM.debug("kQuadratic", coefficients[1]);
                telemetryM.debug("kLinear", coefficients[0]);
                telemetryM.update(telemetry);
                telemetryM.debug("Tuning Complete");
                telemetryM.debug("Braking Profile:");
                telemetryM.debug("kQuadraticFriction", coefficients[1]);
                telemetryM.debug("kLinearBraking", coefficients[0]);
                for (BrakeRecord record : brakeData) {
                    Pose p = record.pose;
                    telemetryM.debug(String.format("t=%.0f ms, x=%.2f, y=%.2f, θ=%.2f, v=%.2f",
                            record.timeMs, p.getX(), p.getY(),
                            p.getHeading(),
                            record.velocity));
                }
                telemetryM.update();
                break;
            }
        }
    }
}

class PredictiveBrakingTuner extends OpMode {
    private static final double[] TEST_POWERS =
            {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2};
    private static final double BRAKING_POWER = 0.2;

    private static final int DRIVE_TIME_MS = 1000;

    private enum State {
        START_MOVE,
        WAIT_DRIVE_TIME,
        APPLY_BRAKE,
        WAIT_BRAKE_TIME,
        RECORD,
        DONE
    }

    private static class BrakeRecord {
        double timeMs;
        Pose pose;
        double velocity;

        BrakeRecord(double timeMs, Pose pose, double velocity) {
            this.timeMs = timeMs;
            this.pose = pose;
            this.velocity = velocity;
        }
    }

    private State state = State.START_MOVE;

    private final ElapsedTime timer = new ElapsedTime();

    private int iteration = 0;

    private Vector startPosition;
    private double measuredVelocity;

    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private final List<BrakeRecord> brakeData = new ArrayList<>();

    @Override
    public void init() {}

    @Override
    public void init_loop() {
        telemetryM.debug("The robot will move forwards and backwards starting at max speed and slowing down.");
        telemetryM.debug("Make sure you have enough room. Leave at least 4-5 feet.");
        telemetryM.debug("After stopping, kFriction and kBraking will be displayed.");
        telemetryM.debug("Make sure to turn the timer off.");
        telemetryM.debug("Press B on game pad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        Drawing.drawCurrent();
    }

    @Override
    public void start() {
        timer.reset();
        follower.update();
        follower.startTeleOpDrive(true);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        follower.update();

        if (gamepad1.b) {
            stopRobot();
            requestOpModeStop();
            return;
        }

        double direction = (iteration % 2 == 0) ? 1 : -1;

        switch (state) {
            case START_MOVE: {
                if (iteration >= TEST_POWERS.length) {
                    state = State.DONE;
                    break;
                }

                double currentPower = TEST_POWERS[iteration];
                follower.setMaxPower(currentPower);
                follower.setTeleOpDrive(direction, 0, 0, true);

                timer.reset();
                state = State.WAIT_DRIVE_TIME;
                break;
            }

            case WAIT_DRIVE_TIME: {
                if (timer.milliseconds() >= DRIVE_TIME_MS) {
                    measuredVelocity = follower.getVelocity().getMagnitude();
                    startPosition = follower.getPose().getAsVector();
                    state = State.APPLY_BRAKE;
                }
                break;
            }

            case APPLY_BRAKE: {
                follower.setTeleOpDrive(BRAKING_POWER * direction, 0, 0, true);

                timer.reset();
                state = State.WAIT_BRAKE_TIME;
                break;
            }

            case WAIT_BRAKE_TIME: {
                double t = timer.milliseconds();
                Pose currentPose = follower.getPose();
                double currentVelocity = follower.getVelocity().getMagnitude();

                brakeData.add(new BrakeRecord(t, currentPose, currentVelocity));

                if (follower.getVelocity().dot(new Vector(direction,
                        follower.getHeading())) <= 0) {
                    state = State.RECORD;
                }
                break;
            }

            case RECORD: {
                Vector endPosition = follower.getPose().getAsVector();
                double brakingDistance = endPosition.minus(startPosition).getMagnitude();

                velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});

                telemetryM.debug("Test " + iteration,
                        String.format("v=%.3f  d=%.3f", measuredVelocity,
                                brakingDistance));
                telemetryM.update(telemetry);

                iteration++;
                state = State.START_MOVE;

                break;
            }

            case DONE: {
                stopRobot();

                double[] coefficients = quadraticFit(velocityToBrakingDistance);

                telemetryM.debug("Tuning Complete");
                telemetryM.debug("Braking Profile:");
                telemetryM.debug("kQuadratic", coefficients[1]);
                telemetryM.debug("kLinear", coefficients[0]);
                telemetryM.update(telemetry);
                telemetryM.debug("Tuning Complete");
                telemetryM.debug("Braking Profile:");
                telemetryM.debug("kQuadraticFriction", coefficients[1]);
                telemetryM.debug("kLinearBraking", coefficients[0]);
                for (BrakeRecord record : brakeData) {
                    Pose p = record.pose;
                    telemetryM.debug(String.format("t=%.0f ms, x=%.2f, y=%.2f, θ=%.2f, v=%.2f",
                            record.timeMs, p.getX(), p.getY(),
                            p.getHeading(),
                            record.velocity));
                }
                telemetryM.update();
                break;
            }
        }
    }
}

class AutomatedOffsetsTuner extends OpMode {
    public static double POWER = 0.5;
    public static double RUNTIME = 5;
    private final Timer timer = new Timer();
    private boolean done = false;
    private final Stack<Vector> poses = new Stack<>();
    private Vector offsets;

    private static class Circle {
        Vector center;
        double radius;

        Circle(Vector center, double radius) {
            this.center = center;
            this.radius = radius;
        }
    }

    @Override
    public void init() {
        follower.setStartingPose(new Pose());
        follower.update();
    }

    @Override
    public void start() {
        timer.resetTimer();
        follower.startTeleOpDrive(true);
        follower.setTeleOpDrive(0, 0, POWER, true);
    }

    /**
     * This initializes the PoseUpdater as well as the Panels telemetry.
     */
    @Override
    public void init_loop() {
        telemetryM.debug("This will turn continuously in place for " + RUNTIME + " seconds.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.update(telemetry);
        follower.update();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated offsets and draws the robot.
     */
    @Override
    public void loop() {
        follower.update();

        if (gamepad1.bWasPressed()) {
            follower.setTeleOpDrive(0, 0, 0, true);
            requestOpModeStop();
        }

        if (!done) {
            poses.push(follower.getPose().getAsVector());

            if (timer.getElapsedTimeSeconds() >= RUNTIME) {
                done = true;
                offsets = fitCircle(poses.toArray(new Vector[0]));
                follower.setTeleOpDrive(0, 0, 0, true);
                telemetryM.addData("elapsed time (s)", String.format("%.4f", timer.getElapsedTimeSeconds()));
            } else {
                follower.setTeleOpDrive(0, 0, POWER, true);
                return;
            }
        }

        telemetryM.debug("The following values are the offsets in inches that should be applied to your localizer.");
        telemetryM.debug("strafeX: " + offsets.getXComponent());
        telemetryM.debug("forwardY: " + offsets.getYComponent());
        telemetryM.update(telemetry);
    }

    private Vector fitCircle(Vector[] points) {
        points = Arrays.copyOfRange(points, 20, points.length);
        Circle circle = taubin(points);
        circle = gaussNewton(points, circle);
        return circle.center.times(-1);
    }

    private static Circle taubin(Vector[] pts) {
        int n = pts.length;

        double mx = 0, my = 0;
        for (Vector p : pts) {
            mx += p.getXComponent();
            my += p.getYComponent();
        }
        mx /= n;
        my /= n;

        double Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0;
        for (Vector p : pts) {
            double x = p.getXComponent() - mx, y = p.getYComponent() - my;
            double z = x * x + y * y;
            Mxx += x * x;
            Myy += y * y;
            Mxy += x * y;
            Mxz += x * z;
            Myz += y * z;
            Mzz += z * z;
        }
        Mxx /= n;
        Myy /= n;
        Mxy /= n;
        Mxz /= n;
        Myz /= n;
        Mzz /= n;

        double Mz = Mxx + Myy;
        double CovXy = Mxx * Myy - Mxy * Mxy;
        double VarZ = Mzz - Mz * Mz;

        double A3 = 4 * Mz;
        double A2 = -3 * Mz * Mz - Mzz;
        double A1 = VarZ * Mz + 4 * CovXy * Mz - Mxz * Mxz - Myz * Myz;
        double A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) - VarZ * CovXy;
        double A22 = A2 + A2;
        double A33 = A3 + A3 + A3;

        double x = 0, y = A0;
        for (int iter = 0; iter < 99; iter++) {
            double dy = A1 + x * (A22 + A33 * x);
            double xnew = x - y / dy;
            if (xnew == x || !Double.isFinite(xnew)) break;
            double ynew = A0 + xnew * (A1 + xnew * (A2 + xnew * A3));
            if (Math.abs(ynew) >= Math.abs(y)) break;
            x = xnew;
            y = ynew;
        }

        double det = x * x - x * Mz + CovXy;
        double cxc = (Mxz * (Myy - x) - Myz * Mxy) / det / 2.0;
        double cyc = (Myz * (Mxx - x) - Mxz * Mxy) / det / 2.0;
        double r = Math.sqrt(cxc * cxc + cyc * cyc + Mz);

        Vector center = new Vector();
        center.setOrthogonalComponents(cxc + mx, cyc + my);
        return new Circle(center, r);
    }

    private static Circle gaussNewton(Vector[] pts, Circle init) {
        double a = init.center.getXComponent();
        double b = init.center.getYComponent();
        double r = init.radius;
        int n = pts.length;

        for (int iter = 0; iter < 200; iter++) {
            double[] res = new double[n];
            double[] da = new double[n];
            double[] db = new double[n];
            double[] dr = new double[n];

            for (int i = 0; i < n; i++) {
                double dx = pts[i].getXComponent() - a;
                double dy = pts[i].getYComponent() - b;
                double dist = Math.sqrt(dx * dx + dy * dy);
                if (dist < 1e-4) continue;
                res[i] = dist - r;
                da[i] = -dx / dist;
                db[i] = -dy / dist;
                dr[i] = -1.0;
            }

            double[][] jCols = {da, db, dr};

            Matrix JtJ = new Matrix(3, 3);
            Matrix Jtf = new Matrix(3, 1);

            for (int row = 0; row < 3; row++) {
                double jtf = 0;
                for (int i = 0; i < n; i++) jtf += jCols[row][i] * res[i];
                Jtf.set(row, 0, jtf);

                for (int col = 0; col < 3; col++) {
                    double jtj = 0;
                    for (int i = 0; i < n; i++) jtj += jCols[row][i] * jCols[col][i];
                    JtJ.set(row, col, jtj);
                }
            }

            double gradNorm = 0;
            for (int i = 0; i < 3; i++) gradNorm += Jtf.get(i, 0) * Jtf.get(i, 0);
            if (Math.sqrt(gradNorm) * 2 < 1e-12) break;

            Matrix invJtJ = Matrix.inverse3x3(JtJ);
            Matrix delta = invJtJ.times(Jtf);

            a -= delta.get(0, 0);
            b -= delta.get(1, 0);
            r -= delta.get(2, 0);
        }

        Vector center = new Vector();
        center.setOrthogonalComponents(a, b);
        return new Circle(center, r);
    }
}

/**
 * This is the Drawing class. It handles the drawing of stuff on Panels Dashboard, like the robot.
 *
 * @author Lazar - 19234
 * @version 1.1, 5/19/2025
 */
class Drawing {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    public static void drawCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void drawCurrentAndHistory() {
        Drawing.drawPoseHistory(Tuning.poseHistory);
        drawCurrent();
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    class AutomatedOffsetsTuner extends OpMode {
        public static double POWER = 0.5;
        public static double RUNTIME = 5;
        private final Timer timer = new Timer();
        private boolean done = false;
        private final Stack<Vector> poses = new Stack<>();
        private Vector offsets;

        private static class Circle {
            private final Vector center;
            private final double radius;

            private Circle(Vector center, double radius) {
                this.center = center;
                this.radius = radius;
            }
        }

        @Override
        public void init() {
            follower.setStartingPose(new Pose());
            follower.update();
        }

        @Override
        public void start() {
            timer.resetTimer();
            follower.startTeleOpDrive(true);
            follower.setTeleOpDrive(0, 0, POWER, true);
        }

        /** This initializes the PoseUpdater as well as the Panels telemetry. */
        @Override
        public void init_loop() {
            telemetryM.debug("This will turn continuously in place for " + RUNTIME + " seconds.");
            telemetryM.debug("Make sure you have enough room.");
            telemetryM.update(telemetry);
            follower.update();
        }

        /**
         * This updates the robot's pose estimate, and updates the Panels telemetry with the
         * calculated offsets and draws the robot.
         */
        @Override
        public void loop() {
            follower.update();

            if (gamepad1.bWasPressed()) {
                follower.setTeleOpDrive(0, 0, 0, true);
                requestOpModeStop();
            }

            if (!done) {
                poses.push(follower.getPose().getAsVector());

                if (timer.getElapsedTimeSeconds() >= RUNTIME) {
                    done = true;
                    offsets = fitCircle(poses.toArray(new Vector[0]));
                    follower.setTeleOpDrive(0, 0, 0, true);
                    telemetryM.addData("elapsed time (s)", String.format("%.4f", timer.getElapsedTimeSeconds()));
                } else {
                    follower.setTeleOpDrive(0, 0, POWER, true);
                    return;
                }
            }

            telemetryM.debug("The following values are the offsets in inches that should be applied to your localizer.");
            telemetryM.debug("strafeX: " + offsets.getXComponent());
            telemetryM.debug("forwardY: " + offsets.getYComponent());
            telemetryM.update(telemetry);

            draw();
        }

        private Vector fitCircle(Vector[] points) {
            Circle circle = taubin(points);
            circle = gaussNewton(points, circle);
            return circle.center.times(-1);
        }

        private static Circle taubin(Vector[] pts) {
            int n = pts.length;

            // Mean-center for numerical stability
            double mx = 0, my = 0;
            for (Vector p : pts) { mx += p.getXComponent(); my += p.getYComponent(); }
            mx /= n; my /= n;

            // Accumulate moments in centered coordinates (u,v) = (x-mx, y-my)
            double Muu=0, Mvv=0, Muv=0, Muz=0, Mvz=0, Mzz=0;
            for (Vector p : pts) {
                double u = p.getXComponent()-mx, v = p.getYComponent()-my;
                double z = u*u + v*v;
                Muu += u*u; Mvv += v*v; Muv += u*v;
                Muz += u*z; Mvz += v*z; Mzz += z*z;
            }
            Muu/=n; Mvv/=n; Muv/=n; Muz/=n; Mvz/=n; Mzz/=n;
            double Mz = Muu + Mvv;

            // Data scatter matrix (symmetric by construction)
            Matrix Cov = new Matrix(new double[][]{
                    { Mzz, Muz, Mvz },
                    { Muz, Muu, Muv },
                    { Mvz, Muv, Mvv }
            });

            // Taubin constraint matrix:
            //   Con = [[4Mz, 0, 2],
            //          [0,   1, 0],
            //          [2,   0, 0]]
            //
            // Analytic Cholesky factor L (lower triangular, Con = L L^T):
            //   L    = [[2*sqMz,  0,      0     ],
            //           [0,       1,      0     ],
            //           [1/sqMz,  0,      1/sqMz]]
            //
            // Analytic L^{-1} (Linv, lower triangular):
            //   Linv = [[1/(2sqMz),  0,   0    ],
            //           [0,          1,   0    ],
            //           [-1/(2Mz),   0,   sqMz ]]
            double sqMz = Math.sqrt(Mz);
            double li00 =  1.0/(2*sqMz);   // Linv[0][0]
            double li11 =  1.0;            // Linv[1][1]
            double li20 = -1.0/(2*Mz);    // Linv[2][0]
            double li22 =  sqMz;          // Linv[2][2]

            // S = Linv * Cov * Linv^T  via two sparse matrix multiplies.
            // Linv is lower-triangular with only 4 nonzeros, so we expand manually
            // rather than paying for a general Matrix.multiply — but we use Matrix
            // to store the result so the rest of the code can use Matrix operations.
            Matrix tmp = new Matrix(3, 3);  // tmp = Linv * Cov
            for (int j = 0; j < 3; j++) {
                tmp.set(0, j, li00 * Cov.get(0, j));
                tmp.set(1, j, li11 * Cov.get(1, j));
                tmp.set(2, j, li20 * Cov.get(0, j) + li22 * Cov.get(2, j));
            }
            Matrix S = new Matrix(3, 3);    // S = tmp * Linv^T
            for (int i = 0; i < 3; i++) {
                // Linv^T columns: col0=(li00,0,li20), col1=(0,li11,0), col2=(0,0,li22)
                S.set(i, 0, tmp.get(i,0)*li00 + tmp.get(i,2)*li20);
                S.set(i, 1, tmp.get(i,1)*li11);
                S.set(i, 2, tmp.get(i,2)*li22);
            }
            // Force exact symmetry to kill floating-point residual asymmetry
            for (int i = 0; i < 3; i++)
                for (int j = i+1; j < 3; j++) {
                    double avg = (S.get(i,j) + S.get(j,i)) * 0.5;
                    S.set(i,j,avg); S.set(j,i,avg);
                }

            // Analytic symmetric eigensolver — valid because S is symmetric by construction
            double[]   eigval = new double[3];
            double[][] eigvec = new double[3][3];
            eigenSymm3x3(S, eigval, eigvec);

            // Map eigenvectors back to original space: p = Linv^T * q
            // Linv^T is upper triangular:  [[li00, 0, li20], [0, li11, 0], [0, 0, li22]]
            for (int k = 0; k < 3; k++) {
                double q0 = eigvec[0][k], q1 = eigvec[1][k], q2 = eigvec[2][k];
                eigvec[0][k] = li00*q0 + li20*q2;
                eigvec[1][k] = li11*q1;
                eigvec[2][k] = li22*q2;
            }

            // Pick eigenvector with smallest positive eigenvalue
            int best = -1;
            double bestVal = Double.MAX_VALUE;
            for (int i = 0; i < 3; i++)
                if (eigval[i] > 1e-10 && eigval[i] < bestVal) { bestVal = eigval[i]; best = i; }
            if (best < 0) best = 0;

            double A0 = eigvec[0][best], B0 = eigvec[1][best], C0 = eigvec[2][best];
            double cx = -B0/(2*A0) + mx;
            double cy = -C0/(2*A0) + my;
            double r  = Math.sqrt(cx*cx + cy*cy - C0/A0);

            Vector center = new Vector();
            center.setOrthogonalComponents(cx, cy);
            return new Circle(center, r);
        }

        private static void eigenSymm3x3(Matrix S, double[] eigval, double[][] eigvec) {
            double s00=S.get(0,0), s11=S.get(1,1), s22=S.get(2,2);
            double s01=S.get(0,1), s02=S.get(0,2), s12=S.get(1,2);

            double tr = s00 + s11 + s22;
            double q  = tr / 3.0;

            // p = sqrt( ||S - q*I||_F^2 / 6 )
            double p2 = (s00-q)*(s00-q) + (s11-q)*(s11-q) + (s22-q)*(s22-q)
                    + 2*(s01*s01 + s02*s02 + s12*s12);
            double p  = Math.sqrt(p2 / 6.0);

            if (p < 1e-14) {
                // S is a scalar multiple of I — all eigenvalues equal q
                eigval[0] = eigval[1] = eigval[2] = q;
                eigvec[0][0]=eigvec[1][1]=eigvec[2][2]=1;
                return;
            }

            // B = (S - q*I) / p,  r = det(B) / 2
            double b00=(s00-q)/p, b11=(s11-q)/p, b22=(s22-q)/p;
            double b01=s01/p,     b02=s02/p,     b12=s12/p;
            double r = ( b00*(b11*b22 - b12*b12)
                    - b01*(b01*b22 - b12*b02)
                    + b02*(b01*b12 - b11*b02) ) / 2.0;

            // Clamp to [-1,1] for numerical safety in acos
            double phi = r <= -1.0 ? Math.PI/3.0
                    : r >=  1.0 ? 0.0
                    : Math.acos(r) / 3.0;

            eigval[0] = q + 2*p*Math.cos(phi);
            eigval[2] = q + 2*p*Math.cos(phi + 2*Math.PI/3.0);
            eigval[1] = tr - eigval[0] - eigval[2];  // trace identity — more stable than third cos

            // Eigenvectors via cross products of rows of (S - lambda*I)
            for (int k = 0; k < 3; k++) {
                double lam = eigval[k];
                double[] r0 = { s00-lam, s01,     s02     };
                double[] r1 = { s01,     s11-lam, s12     };
                double[] r2 = { s02,     s12,     s22-lam };

                double[] v = maxNorm3(cross3(r0, r1), cross3(r0, r2), cross3(r1, r2));
                double norm = Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
                eigvec[0][k] = v[0]/norm;
                eigvec[1][k] = v[1]/norm;
                eigvec[2][k] = v[2]/norm;
            }
        }

        // Gauss-Newton minimizing sum of (dist_i - r)^2
        private static Circle gaussNewton(Vector[] pts, Circle init) {
            double a = init.center.getXComponent();
            double b = init.center.getYComponent();
            double r = init.radius;
            int n = pts.length;

            for (int iter = 0; iter < 200; iter++) {
                // Residuals f_i = dist_i - r  and Jacobian rows [df/da, df/db, df/dr]
                double[] res = new double[n];
                double[] da  = new double[n];
                double[] db  = new double[n];
                double[] dr  = new double[n];

                for (int i = 0; i < n; i++) {
                    double dx   = pts[i].getXComponent() - a;
                    double dy   = pts[i].getYComponent() - b;
                    double dist = Math.sqrt(dx*dx + dy*dy);
                    res[i] = dist - r;
                    da[i]  = -dx/dist;
                    db[i]  = -dy/dist;
                    dr[i]  = -1.0;
                }

                // Build J^T J (3x3) and J^T f (3x1) using Matrix
                // J columns are [da, db, dr] treated as column vectors
                double[][] jCols = { da, db, dr };

                Matrix JtJ = new Matrix(3, 3);
                Matrix Jtf = new Matrix(3, 1);

                for (int row = 0; row < 3; row++) {
                    double jtf = 0;
                    for (int i = 0; i < n; i++) jtf += jCols[row][i] * res[i];
                    Jtf.set(row, 0, jtf);

                    for (int col = 0; col < 3; col++) {
                        double jtj = 0;
                        for (int i = 0; i < n; i++) jtj += jCols[row][i] * jCols[col][i];
                        JtJ.set(row, col, jtj);
                    }
                }

                // Convergence check: gradient of cost = 2 * J^T f
                double gradNorm = 0;
                for (int i = 0; i < 3; i++) gradNorm += Jtf.get(i,0) * Jtf.get(i,0);
                if (Math.sqrt(gradNorm) * 2 < 1e-12) break;

                // Solve JtJ * delta = Jtf using Matrix.rref
                Matrix[] rref = Matrix.rref(JtJ, Jtf);
                Matrix delta = rref[1];  // solution after back-substitution

                a -= delta.get(0, 0);
                b -= delta.get(1, 0);
                r -= delta.get(2, 0);
            }

            Vector center = new Vector();
            center.setOrthogonalComponents(a, b);
            return new Circle(center, r);
        }

        private static double[] cross3(double[] a, double[] b) {
            return new double[]{
                    a[1]*b[2] - a[2]*b[1],
                    a[2]*b[0] - a[0]*b[2],
                    a[0]*b[1] - a[1]*b[0]
            };
        }

        private static double[] maxNorm3(double[]... vecs) {
            double[] best = vecs[0]; double bestN = 0;
            for (double[] v : vecs) {
                double n = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
                if (n > bestN) { bestN = n; best = v; }
            }
            return best;
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}