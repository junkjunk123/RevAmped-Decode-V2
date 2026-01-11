package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.Globals;

@Disabled
@TeleOp
@Config
public class FlywheelPIDTuner extends OpMode {
    private Flywheel flywheel;
    public static int targetVelocity;
    private double totalError = 0;
    private int sampleAmount = 0;
    public static double P = 0.0001;
    public static double K_static = 0.03;
    public static double K_V = 0.0005;
    public static double L = 0.3;
    private GamepadEx gamepad_1;
    private final ElapsedTime timer = new ElapsedTime();
//
    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Globals.telemetry = telemetry;
        gamepad_1 = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        flywheel.runToVel(targetVelocity);
        double vel = flywheel.getVelocityImperial();
        sampleAmount++;
        telemetry.addData("error", targetVelocity - vel);
        telemetry.addData("power", flywheel.getPower());
        telemetry.addData("vel", vel);
        telemetry.addData("average Error", totalError / sampleAmount);
        //telemetry.addData("samples",sampleAmount);

        if (gamepad1.aWasPressed()) {
            flywheel.setPower(1.0);
        }

        if (gamepad1.bWasPressed()) {
            flywheel.setPower(0.0);
        }

        telemetry.addData("vel", flywheel.getVelocityImperial());

        flywheel.update();
        telemetry.update();
        gamepad_1.update();
    }

    @Override
    public void start() {
        timer.reset();
    }

}

