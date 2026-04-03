package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.firstinspires.ftc.teamcode.pedro.Tuning.follower;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.utils.SquIDBrakingController;

@Configurable
@TeleOp
public class Line extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    private final Pose interPose = new Pose(24 + 72, -24 + 72, Math.toRadians(90));
    private final Pose endPose = new Pose(24 + 72, 24 + 72, Math.toRadians(45));

    private PathChain triangle;

    public static double FORWARD_P = 0.15;
    public static double BACKWARD_P = 0.15;

    TelemetryManager telemetryM;
    Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetryM.debug("This will activate all the PIDF(s)");
        telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
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

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        follower.update();
        follower.vectorCalculator.predictiveBrakingController.setCoefficients(new PredictiveBrakingCoefficients(
                FORWARD_P,
                0.0838,
                0.00108
        ));

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

        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }

        telemetryM.debug("Driving Forward?: " + forward);
        telemetryM.debug("forward_P", FORWARD_P);
        telemetryM.debug("backward_P", BACKWARD_P);
        telemetryM.update(telemetry);
    }
}