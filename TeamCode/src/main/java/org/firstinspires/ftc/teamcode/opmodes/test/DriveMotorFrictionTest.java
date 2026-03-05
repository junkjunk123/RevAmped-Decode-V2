package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Disabled
@Config
@TeleOp
public class DriveMotorFrictionTest extends OpMode {
    private Follower follower;
    public static double power;
    public static double increment;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        increment = 0.1;
    }

    @Override
    public void loop() {
        telemetry.addData("Increment",increment);
        telemetry.addData("Current Power",power);
        if (gamepad1.aWasPressed()) power += increment;
        if (gamepad1.bWasPressed()) power -= increment;
        if (gamepad1.leftBumperWasPressed()) increment/=10;
        if (gamepad1.rightBumperWasPressed()) increment*=10;

        follower.drivetrain.runDrive(new double[] {power, power, power, power});
        follower.update();
    }
}
