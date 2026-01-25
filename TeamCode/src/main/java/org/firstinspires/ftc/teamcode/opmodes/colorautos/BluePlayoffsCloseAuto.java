package org.firstinspires.ftc.teamcode.opmodes.colorautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.CloseAuto;
import org.firstinspires.ftc.teamcode.opmodes.UnsortedCloseAuto;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

@Autonomous
public class BluePlayoffsCloseAuto extends UnsortedCloseAuto {
    @Override
    public void initialize() {
        Globals.allianceColor = AllianceColor.Blue;
        super.initialize();
    }
}
