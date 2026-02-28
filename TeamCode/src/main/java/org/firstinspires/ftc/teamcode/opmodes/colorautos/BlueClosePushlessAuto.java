package org.firstinspires.ftc.teamcode.opmodes.colorautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.CloseAuto;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

@Autonomous
public class BlueClosePushlessAuto extends CloseAuto {
    @Override
    public void initialize() {
        Globals.setAllianceColor(AllianceColor.Blue);
        shouldPush = false;
        super.initialize();
    }
}
