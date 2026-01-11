package org.firstinspires.ftc.teamcode.mechanisms.octocanum;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class OctocanumBack extends HwServo {
    public OctocanumBack(HardwareMap hardwareMap) throws IllegalArgumentException {
        super(hardwareMap, "octo_back");
    }
    public enum ServoState{
        UP,
        DOWN;
    }

    private ServoState servoState = ServoState.UP;

    public void engage(){
        servoState = ServoState.DOWN;
        setPosition(162/255f);
    }

    public void raise(){
        servoState = ServoState.UP;
        setPosition(56/255f);
    }

    public ServoState getState(){
        return servoState;
    }
}
