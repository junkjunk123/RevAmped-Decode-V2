package org.firstinspires.ftc.teamcode.opmodes.colorautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.EighteenAutoSorted;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

@Autonomous
public class RedEighteenSorted extends EighteenAutoSorted {
    @Override
    public void initialize() {
        Globals.setAllianceColor(AllianceColor.Red);
        super.initialize();
    }
}
