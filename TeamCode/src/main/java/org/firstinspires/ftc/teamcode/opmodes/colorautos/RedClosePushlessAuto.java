package org.firstinspires.ftc.teamcode.opmodes.colorautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodes.CloseAuto;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

@Disabled
@Autonomous
public class RedClosePushlessAuto extends CloseAuto {
    @Override
    public void initialize() {
        Globals.setAllianceColor(AllianceColor.Red);
        shouldPush = false;
        super.initialize();
    }
}
