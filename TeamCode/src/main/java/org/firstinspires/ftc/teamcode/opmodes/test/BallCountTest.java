package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeArtifactDetector;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeThread;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeTilt;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerColorSensors;
import org.firstinspires.ftc.teamcode.utils.commands.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.commands.GamepadEx;

import java.util.Arrays;

@TeleOp
public class BallCountTest extends OpMode {
    IntakeThread intakeThread;
    IntakeMotor intake;
    IntakeTilt intakeTilt;
    IntakeArtifactDetector intakeDistance;
    SpindexerColorSensors colorSensors;
    GamepadEx gamepad_1;
    ArtifactColor[] colors;
    @Override
    public void init() {
        intake = new IntakeMotor(hardwareMap);
        intakeTilt = new IntakeTilt(hardwareMap);
        intakeDistance = new IntakeArtifactDetector(hardwareMap,"intakeDistance",25);
        colorSensors = new SpindexerColorSensors(hardwareMap);
        colors = new ArtifactColor[] {ArtifactColor.NONE, ArtifactColor.NONE,ArtifactColor.NONE};
        intakeThread = new IntakeThread(colors,colorSensors,intakeDistance);
        gamepad_1 = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        if (gamepad_1.a.isRisingEdge()){
            intakeThread.reset();
        }
        if (gamepad_1.b.isRisingEdge()){
            intake.setPower(1.0);
        }
        if (gamepad_1.y.isRisingEdge()){
            intake.setPower(0.0);
        }
        telemetry.addData("hypothetical",intakeThread.getHypotheticalNumBalls());
        telemetry.addData("num balls",intakeThread.getNumBalls());
        telemetry.addData("has artifact",intakeDistance.hasArtifact());
        telemetry.addData("intake distance",intakeDistance.state());
        telemetry.addData("colors", Arrays.toString(intakeThread.getColors()));
        telemetry.addData("left color",colorSensors.leftColorSensor.getColor());
        telemetry.addData("right color",colorSensors.rightColorSensor.getColor());
        telemetry.addData("state",intakeThread.getDetectionState());
        intakeDistance.update();
        colorSensors.update();
        intakeThread.update();
        gamepad_1.update();
        telemetry.update();
        intake.update();
        intakeTilt.update();
        Scheduler.getInstance().execute();
    }
}
