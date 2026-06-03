package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistanceSensors;

import java.util.Arrays;

@TeleOp
public class DistancePortTest extends OpMode {
    private IntakeDistanceSensors distanceSensors;
    @Override
    public void init() {
        distanceSensors = new IntakeDistanceSensors(hardwareMap);
        distanceSensors.start();
    }

    @Override
    public void loop() {
        telemetry.addData("0,1,2", Arrays.toString(distanceSensors.getStates()));
        distanceSensors.update();
        telemetry.update();
    }
}
