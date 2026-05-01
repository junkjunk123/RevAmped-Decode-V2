package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

@Disabled
@TeleOp
public class servoTurretTest extends OpMode {
    HwServo turret;
    HwServo turret2;
    @Override
    public void init() {
        turret = new HwServo(hardwareMap,"turret");
        turret2 = new HwServo(hardwareMap,"turret2");
        turret.setPosition(0.5);
        turret2.setPosition(0.5);
    }

    @Override
    public void loop() {

    }
}
