package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;

@Autonomous (name = "Blue Alliance 27", group = "Auto")
public class BlueClose27Auto extends Close27Auto {
    @Override
    public void preInit() {
        Globals.setAllianceColor(AllianceColor.Blue);
    }
}
