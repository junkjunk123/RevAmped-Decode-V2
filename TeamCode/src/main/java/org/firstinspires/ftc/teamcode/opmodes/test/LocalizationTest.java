package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.octoquad.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.pedro.octoquad.OctoQuadLocalizer;

@TeleOp
public class LocalizationTest extends OpMode {
    private Follower follower;

    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    public void loop() {
        follower.update();
        telemetry.addData("pose", follower.getPose());

        OctoQuadFWv3 octoquad = ((OctoQuadLocalizer) follower.getPoseTracker().getLocalizer()).getOctoQuad();

        double xPodIn = octoquad.readAllEncoderData().positions[1];
        double yPodIn = octoquad.readAllEncoderData().positions[0];

        double IN_PER_TICK = 0.001989436789;
        xPodIn *= IN_PER_TICK;
        yPodIn *= IN_PER_TICK;

        telemetry.addData("forwardPodY", xPodIn / Math.PI * Math.sqrt(2) / 2.0);
        telemetry.addData("strafePodX", -yPodIn / Math.PI * Math.sqrt(2) / 2.0);
        telemetry.update();
    }
}
