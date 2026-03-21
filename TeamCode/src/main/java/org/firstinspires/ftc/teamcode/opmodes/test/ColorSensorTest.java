package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.DecodeColorSensor;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerColorSensors;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerCompartmentManager;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RandomizationState;
import org.firstinspires.ftc.teamcode.utils.prompter.OptionPrompt;
import org.firstinspires.ftc.teamcode.utils.prompter.Prompter;

@TeleOp
public class ColorSensorTest extends OpMode {
    SpindexerColorSensors colorSensors;
    MultipleTelemetry telemetries;
    @Override
    public void init() {
        colorSensors = new SpindexerColorSensors(hardwareMap,"colorLeft","colorRight");
        telemetries = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        telemetries.addData("Colors",colorSensors.getCompartmentColors());
        colorSensors.update();
    }
}
