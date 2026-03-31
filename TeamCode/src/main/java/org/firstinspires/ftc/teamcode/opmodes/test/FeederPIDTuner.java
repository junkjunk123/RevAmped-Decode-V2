package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.FeederWheel;

@TeleOp
@Config
public class FeederPIDTuner extends OpMode {
    public static double targetVelocity;
    private FeederWheel feederWheel;

    @Override
    public void init() {
        feederWheel = new FeederWheel(hardwareMap);
    }

    @Override
    public void loop() {
        feederWheel.setTargetVelocity(targetVelocity);
        feederWheel.update();

        double vel = feederWheel.getVelocity();

        telemetry.addData("velocity", vel);
        telemetry.addData("error", targetVelocity - vel);
    }

}
