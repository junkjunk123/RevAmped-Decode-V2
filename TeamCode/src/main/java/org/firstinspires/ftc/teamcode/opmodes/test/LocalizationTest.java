package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
@Config
public class LocalizationTest extends OpMode {
    public static double strafePodX = -1.1;
    public static double forwardPodY = -1.1;
    private Follower follower;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("X",follower.getPose().getX());
        telemetry.addData("Y",follower.getPose().getY());
        telemetry.addData("Heading",follower.getPose().getHeading());
        telemetry.update();
        follower.update();
    }
}
