package org.firstinspires.ftc.teamcode.opmodes.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.DecodeColorSensor;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.prompter.OptionPrompt;
import org.firstinspires.ftc.teamcode.utils.prompter.Prompter;

import java.util.List;

@TeleOp
public class ColorSensorCalibrator extends OpMode {
    private Telemetry dashTelemetry;
    private Prompter prompter;
    private GamepadEx gamepad_1;
    private DecodeColorSensor currentSensor;
    private List<String> sensors;

    private enum TestState {
        SELECT,
        CALIBRATE
    }

    private TestState testState = TestState.SELECT;

    @Override
    public void init() {
        gamepad_1 = new GamepadEx(gamepad1);
        sensors = List.of("colorLeft", "colorRight");
        dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        prompter = new Prompter(this, gamepad_1);
        prompter.prompt("sensor", new OptionPrompt<>("Select a sensor -- press right bumper to select", sensors.toArray(new String[0])))
                .onComplete(() -> {
                    currentSensor = new DecodeColorSensor(hardwareMap, prompter.get("sensor"));
                    testState = TestState.CALIBRATE;
                })
                .thenDisplay(() -> "Selected sensor: " + currentSensor);
    }

    @Override
    public void loop() {
        switch (testState) {
            case CALIBRATE -> {
                dashTelemetry.addData("Detection", currentSensor.objectDetected() ? currentSensor.getColor() : "out of range");
                dashTelemetry.addData("Hue", currentSensor.getHue());
                dashTelemetry.addData("Value",currentSensor.getValue());
                dashTelemetry.addData("Saturation",currentSensor.getSaturation());
                dashTelemetry.addData("Distance", currentSensor.getDistanceMM());
                dashTelemetry.update();
                currentSensor.update();
                if(gamepad_1.y.isRisingEdge()){
                    testState = TestState.SELECT;
                }
            }
            case SELECT -> prompter.run();
        }
    }
}
