package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

@Disabled
@TeleOp
public class SetAlliaceBlue extends OpMode {
    @Override
    public void init() {
        Globals.allianceColor = AllianceColor.Blue;
        telemetry.addData("Alliance Set! Current Alliance: ",Globals.allianceColor);
        telemetry.update();
    }

    @Override
    public void loop() {

    }
}
