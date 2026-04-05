package org.firstinspires.ftc.teamcode.opmodes.colorautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.FifteenSorted;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

@Autonomous
public class RedFifteenSorted extends FifteenSorted {
    @Override
    public void initialize() {
        Globals.setAllianceColor(AllianceColor.Red);
        super.initialize();
    }
}
