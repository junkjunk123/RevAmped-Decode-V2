package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;

public class TurretSim extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) robot.turret.setTargetPosition(300);
        if (gamepad1.bWasPressed()) robot.turret.setTargetPosition(0);
        robot.update();
    }

    @Override
    public void stop() {
        Globals.turretStartPos = robot.turret.getPosition();
    }
}
