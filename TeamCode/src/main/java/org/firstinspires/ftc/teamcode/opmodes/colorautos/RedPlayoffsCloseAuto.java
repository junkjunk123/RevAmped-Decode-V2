package org.firstinspires.ftc.teamcode.opmodes.colorautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.UnsortedCloseAuto;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

@Autonomous
public class RedPlayoffsCloseAuto extends UnsortedCloseAuto {
    @Override
    public void initialize() {
        Globals.setAllianceColor(AllianceColor.Red);
        super.initialize();
    }
}
