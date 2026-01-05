package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class LocalizationTest extends OpMode {
    private Follower follower;

    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    public void loop() {
        follower.update();
        telemetry.addData("pose", follower.getPose());

        GoBildaPinpointDriver pinpoint = ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint();

        double xPodIn = pinpoint.getEncoderX();
        double yPodIn = pinpoint.getEncoderY();

        double IN_PER_TICK = 0.001989436789;
        xPodIn *= IN_PER_TICK;
        yPodIn *= IN_PER_TICK;

        telemetry.addData("forwardPodY", xPodIn / Math.PI * Math.sqrt(2) / 2.0);
        telemetry.addData("strafePodX", -yPodIn / Math.PI * Math.sqrt(2) / 2.0);
        telemetry.update();
    }
}
