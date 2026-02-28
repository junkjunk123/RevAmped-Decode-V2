package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.utils.Globals;

@TeleOp
public class HoodReset extends OpMode {
    private Hood hood;

    @Override
    public void init() {
        Globals.init(telemetry);
        hood = new Hood(hardwareMap);
        hood.setPosition(1f);
    }

    @Override
    public void loop() {

    }
}
