package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FusionConstants;
import org.firstinspires.ftc.teamcode.pedro.FusionLocalizer;
import org.firstinspires.ftc.teamcode.utils.hardware.HwUltrasonic;

@TeleOp
public class UltrasonicEKFTest extends OpMode {
    private Follower follower;
    private HwUltrasonic leftSensor;
    private HwUltrasonic rightSensor;
    private static double TRACK_WIDTH;

    @Override
    public void init() {
        FusionConstants constants = new FusionConstants();
        follower = new FollowerBuilder(Constants.followerConstants, hardwareMap)
                .mecanumDrivetrain(Constants.driveConstants)
                .pathConstraints(Constants.pathConstraints)
                .setLocalizer(new FusionLocalizer(new PinpointLocalizer(hardwareMap, Constants.localizerConstants),
                        constants))
                .build();
        leftSensor = new HwUltrasonic(hardwareMap, "leftUltrasonic");
        rightSensor = new HwUltrasonic(hardwareMap, "rightUltrasonic");
    }

    @Override
    public void loop() {
        follower.update();
        double leftXDist = leftSensor.getDistance();
        double rightXDist = rightSensor.getDistance();
        double sensorXPos = (leftXDist + rightXDist) / 2;
        double sensorHeading = Math.atan2(TRACK_WIDTH, leftXDist - rightXDist);

        ((FusionLocalizer) follower.getPoseTracker().getLocalizer()).addMeasurement(
                new Pose(sensorXPos, Double.NaN, sensorHeading),
                System.nanoTime()
        );

        telemetry.addData("ultrasonicX", sensorXPos);
        telemetry.addData("ultrasonicHeading", sensorHeading);
        telemetry.addData("pinpointX", follower.getPose().getX());
        telemetry.addData("pinpointHeading", follower.getPose().getHeading());
    }
}
