package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TurretCalibrater extends OpMode {
    private int base;
    private DcMotorEx turret;
    private DcMotorEx leftFront;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "motor_lf");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        base = leftFront.getCurrentPosition();
    }

    @Override
    public void loop() {
        telemetry.addData("pos", leftFront.getCurrentPosition() - base);
        telemetry.update();
    }
}
