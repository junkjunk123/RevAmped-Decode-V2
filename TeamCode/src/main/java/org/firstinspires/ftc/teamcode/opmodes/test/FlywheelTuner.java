package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Globals;

@Config
@TeleOp
public class FlywheelTuner extends OpMode {
    private Flywheel flywheel;
    public static float targetVelocity = 1000;
    private final ElapsedTime timer = new ElapsedTime();
    private double time = 0;
    private boolean measured = false;

    @Override
    public void init() {
        Globals.constants.build();
        flywheel = new Flywheel(hardwareMap);
        flywheel.runToVel(targetVelocity);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        flywheel.update();

        if (flywheel.getError() < 50 && !measured) {
            time = timer.milliseconds();
            measured = true;
        }

        telemetry.addData("target", flywheel.getTargetVelocity());
        telemetry.addData("error", flywheel.getError());
        telemetry.addData("flywheelVel", flywheel.getVelocityImperial());
        telemetry.addData("flywheelFilteredVel", flywheel.getFilteredVelocity());
        telemetry.addData("time", time);
        telemetry.update();
    }
}
