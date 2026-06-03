package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
@Disabled
public class ServoTest extends OpMode {
    private Servo servo0;
    public static double initialPos = 1/4d;
    public static double finalPos = 3/4d;

    @Override
    public void init() {
        servo0 = hardwareMap.get(Servo.class, "c-5");
        servo0.setPosition(initialPos);
    }

    @Override
    public void start() {
        servo0.setPosition(finalPos);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            servo0.setPosition(initialPos);
        }
    }
}
