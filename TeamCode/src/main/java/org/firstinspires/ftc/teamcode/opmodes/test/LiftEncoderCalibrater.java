package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.lift.Lift;

public class LiftEncoderCalibrater extends OpMode {
    private Lift lift;
    private IntakeMotor intake;

    @Override
    public void init() {
        intake = new IntakeMotor(hardwareMap);
        lift = new Lift(hardwareMap, intake.getEncoder());
    }

    @Override
    public void loop() {
        lift.update();

        telemetry.addData("pos", lift.getPosition());
        telemetry.addData("raw", lift.getEncoder().getPosition());
        telemetry.addData("base", lift.getEncoderBase());
        telemetry.update();
    }
}
