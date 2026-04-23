package org.firstinspires.ftc.teamcode.opmodes.colorautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.EighteenAutoSortedRed;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;

@Autonomous
public class RedEighteenSorted extends EighteenAutoSortedRed {
    @Override
    public void preInit() {
        Globals.setAllianceColor(AllianceColor.Red);
    }
}
