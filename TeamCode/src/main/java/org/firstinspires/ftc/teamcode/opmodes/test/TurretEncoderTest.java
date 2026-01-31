package org.firstinspires.ftc.teamcode.opmodes.test;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;

@Config
@TeleOp
public class TurretEncoderTest extends OpMode {
    public static int mult = 1;
    private Turret turret;
    private boolean searchingSwitch;
    private boolean searchingEncoder;
    private double timeSwitch;
    private double timeEncoder;
    private double timePredicted;
    private double startTime;

    @Override
    public void init() {
        Globals.init(telemetry);
        turret = new Turret(hardwareMap, Encoder.fromMotor(hardwareMap.get(DcMotorEx.class, "motor_lf")));
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            turret.setTargetPosition(mult * 400);
            searchingSwitch = true;
            searchingEncoder = true;
            startTime = System.currentTimeMillis();
        }

        if (gamepad1.bWasPressed()) {
            turret.setTargetPosition(0);
            searchingSwitch = true;
            searchingEncoder = true;
            startTime = System.currentTimeMillis();
            timePredicted = turret.predictedMoveTime();
        }

        if (searchingSwitch && turret.limitSwitch.getReading()) {
            searchingSwitch = false;
            timeSwitch = System.currentTimeMillis() - startTime;
        }

        if (searchingEncoder && Math.abs(turret.getVelocity()) < 10 && Math.abs(turret.getTargetPosition() - turret.getPosition()) < 25) {
            searchingEncoder = false;
            timeEncoder = System.currentTimeMillis() - startTime;
        }

        if (gamepad1.xWasPressed()) {
            Scheduler.getInstance().schedule(turret.resetTurret());
        }

        if (gamepad1.yWasPressed()) {
            turret.resetPosition(mult * 300);
            turret.setTargetPosition(mult * 300);
        }

        telemetry.addData("turretPos", turret.getPosition());
        telemetry.addData("targetPos", turret.getTargetPosition());
        telemetry.addData("switch", turret.limitSwitch.get());
        telemetry.addData("timeSwitch", timeSwitch);
        telemetry.addData("timeEncoder", timeEncoder);
        telemetry.addData("timePredicted", timePredicted);
        telemetry.addData("turretVel", turret.getVelocity());

        Log.e("switch", String.valueOf(turret.limitSwitch.get()));
        Scheduler.getInstance().execute();
        turret.update();
        telemetry.update();
    }
}
