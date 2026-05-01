package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerColorSensors;

import java.util.Arrays;

@Disabled
@TeleOp
public class ColorSensorTest extends OpMode {
    SpindexerColorSensors colorSensors;
    MultipleTelemetry telemetries;
    @Override
    public void init() {
        colorSensors = new SpindexerColorSensors(hardwareMap);
        telemetries = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        telemetries.addData("Colors", Arrays.toString(colorSensors.getCompartmentColors()));
        colorSensors.update();
        colorSensors.updateColors();
        telemetries.update();
    }
}
