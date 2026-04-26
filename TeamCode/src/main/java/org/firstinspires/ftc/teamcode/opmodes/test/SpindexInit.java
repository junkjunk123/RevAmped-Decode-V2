package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;

@TeleOp
public class SpindexInit extends OpMode {
    private Robot robot;
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.table.setStateCommandless(Table.RelativeState.BALL1);
    }

    @Override
    public void loop() {

    }
}
