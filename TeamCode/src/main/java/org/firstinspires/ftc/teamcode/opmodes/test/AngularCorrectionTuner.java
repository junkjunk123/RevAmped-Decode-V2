package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.GyroThread;
import org.firstinspires.ftc.teamcode.utils.math.projectile.TrackState;

@Disabled
@TeleOp
public class AngularCorrectionTuner extends OpMode {
    private GyroThread gyroThread;
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        gyroThread = new GyroThread(robot);
        gyroThread.setState(TrackState.FAR_ONE);
    }

    @Override
    public void loop() {
        robot.drivetrain.arcadeDrive(gamepad1);
        robot.update();
        gyroThread.update(true);
    }
}
