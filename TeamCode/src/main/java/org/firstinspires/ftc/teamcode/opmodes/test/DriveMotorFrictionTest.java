package org.firstinspires.ftc.teamcode.opmodes.test;
import static org.firstinspires.ftc.teamcode.pedro.Tuning.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp
public class DriveMotorFrictionTest extends OpMode {
    private Follower follower;
    public static double power;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) power += 0.1;
        if (gamepad1.bWasPressed()) power -= 0.1;

        follower.drivetrain.runDrive(new double[] {power, power, power, power});
        follower.update();
    }
}
