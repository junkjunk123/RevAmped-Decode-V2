package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

@TeleOp
public class SetAllianceRed extends OpMode {
    @Override
    public void init() {
        Globals.allianceColor = AllianceColor.Red;
        telemetry.addData("Alliance Set! Current Alliance: ",Globals.allianceColor);
        telemetry.update();
    }

    @Override
    public void loop() {

    }
}
