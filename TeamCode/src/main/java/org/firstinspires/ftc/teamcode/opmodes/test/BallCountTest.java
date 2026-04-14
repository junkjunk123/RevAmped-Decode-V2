package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeArtifactDetector;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerColorSensors;
import org.firstinspires.ftc.teamcode.utils.commands.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.commands.GamepadEx;

import java.util.Arrays;

@TeleOp
public class BallCountTest extends OpMode {
    IntakeThread intakeThread;
    IntakeMotor intake;
    IntakeArtifactDetector intakeDistance;
    SpindexerColorSensors colorSensors;
    GamepadEx gamepad_1;
    ArtifactColor[] colors;
    @Override
    public void init() {
        intake = new IntakeMotor(hardwareMap);
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
        telemetry.addData("hypothetical",intakeThread.getHypotheticalNumBalls());
        telemetry.addData("has artifact",intakeDistance.hasArtifact());
        telemetry.addData("intake distance",intakeDistance.state());
        telemetry.addData("colors", Arrays.toString(intakeThread.getColors()));
        intakeDistance.update();
        colorSensors.update();
        intakeThread.update();
        gamepad_1.update();
        telemetry.update();
        Scheduler.getInstance().execute();
    }
}
