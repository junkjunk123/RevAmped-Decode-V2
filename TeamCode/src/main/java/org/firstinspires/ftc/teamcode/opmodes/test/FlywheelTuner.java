package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Globals;

//@Disabled
@Config
@TeleOp
@Disabled
public class FlywheelTuner extends OpMode {
    private Flywheel flywheel;
    public static float targetVelocity = 1000;
    private final ElapsedTime timer = new ElapsedTime();
    private double time = 0;
    private boolean measured = false;
    public static boolean track;
    public static boolean reset;
    private double minVel = Double.MAX_VALUE;

    @Override
    public void init() {
        Globals.constants.build();
        flywheel = new Flywheel(hardwareMap);
        flywheel.setVelocity(targetVelocity);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        flywheel.update();

        if (flywheel.getError() < 20 && !measured) {
            time = timer.milliseconds();
            measured = true;
        }

        if (targetVelocity != flywheel.getTargetVelocity()) {
            flywheel.setVelocity(targetVelocity);
            timer.reset();
        }

        if (track) {
            minVel = Math.min(minVel, flywheel.getFilteredVelocity());
        }

        if (reset) {
            reset = false;
            timer.reset();
            measured = false;
        }

        telemetry.addData("target", flywheel.getTargetVelocity());
        telemetry.addData("error", flywheel.getError());
        telemetry.addData("flywheelVel", flywheel.getVelocityImperial());
        telemetry.addData("flywheelFilteredVel", flywheel.getFilteredVelocity());
        telemetry.addData("minVel", minVel);
        telemetry.addData("time", time);
        telemetry.addData("power", flywheel.getPower());
        telemetry.update();
    }
}