package org.firstinspires.ftc.teamcode.opmodes.colorautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.CloseAuto;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;

@Autonomous
public class BlueCloseAuto extends CloseAuto {
    @Override
    public void preInit() {
        Globals.setAllianceColor(AllianceColor.Blue);
    }
}