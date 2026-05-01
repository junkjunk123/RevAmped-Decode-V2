package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeArtifactDetector;

@Disabled
@TeleOp
public class IntakeDistanceTest extends OpMode {
    IntakeArtifactDetector intakeDistance;
    @Override
    public void init() {
        intakeDistance = new IntakeArtifactDetector(hardwareMap,"intakeDistance",25);
    }

    @Override
    public void loop() {
        telemetry.addData("reading",intakeDistance.state());
        telemetry.update();
        intakeDistance.update();
    }
}
