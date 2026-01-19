package org.firstinspires.ftc.teamcode.mechanisms.octocanum;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class OctocanumBack extends HwServo {
    public OctocanumBack(HardwareMap hardwareMap) throws IllegalArgumentException {
        super(hardwareMap, "octo_back");
    }

    public static float ENGAGED;
    public static float RAISED;

    public void engage(){
        setPosition(ENGAGED);
    }

    public void raise(){
        setPosition(RAISED);
    }
}
