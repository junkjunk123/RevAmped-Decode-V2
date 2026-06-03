package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretMTI;
import org.firstinspires.ftc.teamcode.utils.commands.GamepadEx;
import org.firstinspires.ftc.teamcode.utils.prompter.OptionPrompt;
import org.firstinspires.ftc.teamcode.utils.prompter.Prompter;

import java.util.HashMap;
import java.util.List;

@Config
@TeleOp(name = "TurretCalibrator", group = "a")
public class TurretCalibrator extends OpMode {
    private int posJoy1;
    private long timeStamp = 0;
    public static int denominator = 255;
    public static int TICK_CHANGE = 1;
    private boolean singleTickMode = false;
    private Telemetry telemetryA;
    private GamepadEx gamepad_1;
    private ServoTurretMTI servoTurret;

    @Override
    public void init() {
        gamepad_1 = new GamepadEx(gamepad1);
        servoTurret = new ServoTurretMTI(hardwareMap);
        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();
    }

    @Override
    public void loop() {
        gamepad_1.update();
        timeStamp = System.currentTimeMillis();
        if (!singleTickMode) {
            if (gamepad_1.x.canPress4Short(timeStamp)) {
                posJoy1 -= TICK_CHANGE;
            } else if (gamepad_1.b.canPress4Short(timeStamp)) {
                posJoy1 += TICK_CHANGE;
            }
        } else {
            if (gamepad_1.x.canPress(timeStamp)) {
                posJoy1 -= TICK_CHANGE;
            } else if (gamepad_1.b.canPress(timeStamp)) {
                posJoy1 += TICK_CHANGE;
            }
        }

        if (gamepad_1.y.canPress(timeStamp)) {
            singleTickMode = !singleTickMode;
        } else if (gamepad_1.right_stick_button.canPress(timeStamp)) {
            posJoy1 = 128;
        }

        posJoy1 = Range.clip(posJoy1, 0, denominator);
        servoTurret.setPosition((float) posJoy1 / denominator);

        if (gamepad_1.right_bumper.isRisingEdge()) {
            TICK_CHANGE += 5;
            TICK_CHANGE = Range.clip(TICK_CHANGE, 1, 100);
        } else if (gamepad_1.left_bumper.isRisingEdge()) {
            TICK_CHANGE -= 5;
            TICK_CHANGE = Range.clip(TICK_CHANGE, 1, 100);
        } else if (gamepad_1.right_stick_button.isRisingEdge()) {
            TICK_CHANGE = 1;
        }
        else {
            telemetryA.addData("pos", posJoy1);
        }
        telemetryA.addData("singleTickMode", singleTickMode);
        telemetryA.addData("TICK_CHANGE", TICK_CHANGE);
        telemetryA.update();
    }
}
