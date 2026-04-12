package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerColorSensors;
import org.firstinspires.ftc.teamcode.utils.commands.ArtifactColor;

@TeleOp
public class ColorTest extends OpMode {
    private SpindexerColorSensors sensors;

    @Override
    public void init() {
        sensors = new SpindexerColorSensors(hardwareMap);
    }

    @Override
    public void loop() {
        double leftHue = sensors.leftColorSensor.getHue();
        double leftDist = sensors.leftColorSensor.getDistanceMM();
        double rightHue = sensors.rightColorSensor.getHue();
        double rightDist = sensors.rightColorSensor.getDistanceMM();
        ArtifactColor leftColor = sensors.leftColorSensor.getColor();
        ArtifactColor rightColor = sensors.rightColorSensor.getColor();

        telemetry.addData("leftHue", leftHue);
        telemetry.addData("leftDist", leftDist);
        telemetry.addData("rightHue", rightHue);
        telemetry.addData("rightDist", rightDist);
        telemetry.addData("leftColor", leftColor);
        telemetry.addData("rightColor", rightColor);
        telemetry.update();
    }
}
