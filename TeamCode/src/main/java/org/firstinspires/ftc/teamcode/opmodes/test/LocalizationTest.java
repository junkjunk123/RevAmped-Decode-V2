package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.octoquad.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.pedro.octoquad.OctoQuadLocalizer;

@Disabled
@Config
@TeleOp
public class LocalizationTest extends OpMode {
    private Follower follower;
    private double initialX;
    private double initialY;
    public static double forwardY = 4;
    public static double strafeX = -2;

    public void init() {
        follower = Constants.createFollower(hardwareMap);
        GoBildaPinpointDriver pinpoint = ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint();
        follower.update();
        initialX = pinpoint.getEncoderX();
        initialY = pinpoint.getEncoderY();
    }

    public void loop() {
        follower.update();
        telemetry.addData("pose", follower.getPose());

        GoBildaPinpointDriver pinpoint = ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint();

        double xPodIn = pinpoint.getEncoderX() - initialX;
        double yPodIn = pinpoint.getEncoderY() - initialY;

        double IN_PER_TICK = 0.001989436789;
        xPodIn *= IN_PER_TICK;
        yPodIn *= IN_PER_TICK;

        telemetry.addData("xPod", pinpoint.getEncoderX() - initialX);
        telemetry.addData("yPod", pinpoint.getEncoderY() - initialY);
        telemetry.addData("xPodInch", xPodIn);
        telemetry.addData("yPodInch", yPodIn);
        telemetry.addData("forwardPodY", xPodIn / Math.PI * Math.sqrt(2) / 2.0);
        telemetry.addData("strafePodX", -yPodIn / Math.PI * Math.sqrt(2) / 2.0);
        telemetry.update();
    }
}
