package org.firstinspires.ftc.teamcode.opmodes.colorautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodes.FarAuto;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;

@Disabled
@Autonomous
public class RedFarAuto extends FarAuto {
    public RedFarAuto() {
        super(AllianceColor.Red);
    }
}
