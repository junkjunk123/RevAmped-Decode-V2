package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.FeederWheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.utils.Globals;

@TeleOp
@Config
public class FeederPIDTuner extends OpMode {
    public static double targetVelocity;
    private FeederWheel feederWheel;
    private IntakeMotor intake;
    private Flywheel flywheel;

    @Override
    public void init() {
        Globals.init(telemetry);
        feederWheel = new FeederWheel(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new IntakeMotor(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        flywheel.near();
        intake.intake();
    }

    @Override
    public void loop() {
        feederWheel.setTargetVelocity(targetVelocity);
        flywheel.update();
        intake.update();
        feederWheel.update();

        double vel = feederWheel.getVelocity();

        telemetry.addData("velocity", vel);
        telemetry.addData("error", targetVelocity - vel);
        telemetry.update();
    }

}
