package org.firstinspires.ftc.teamcode.mechanisms.lift;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwCRServo;

public class Tilt extends HwCRServo {
    public static double TIME;

    public Tilt(HardwareMap hwMap) {
        super(hwMap, "tilt");
    }

    public ICommand lift() {
        return new Sequential(
                new Instant(() -> setPower(1)),
                new Wait(TIME),
                new Instant(() -> setPower(0))
        );
    }
}
