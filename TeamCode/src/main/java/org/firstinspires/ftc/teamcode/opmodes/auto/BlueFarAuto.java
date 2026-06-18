package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;

@Autonomous
public class BlueFarAuto extends FarAuto {
    @Override
    public void preInit() {
        Globals.setAllianceColor(AllianceColor.Blue);
    }
}
