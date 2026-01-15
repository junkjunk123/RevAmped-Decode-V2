package org.firstinspires.ftc.teamcode.mechanisms.octocanum;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class OctocanumFront extends HwServo {
    public OctocanumFront(HardwareMap hardwareMap) throws IllegalArgumentException {
        super(hardwareMap, "octo_front");
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
