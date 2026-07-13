package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;

@Disabled
@Autonomous(name = "RedAuto")
public class RedSolo27 extends SoloCloseAuto {
    @Override
    public void preInit() {
        Globals.setAllianceColor(AllianceColor.Red);
    }
}
