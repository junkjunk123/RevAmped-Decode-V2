package org.firstinspires.ftc.teamcode.mechanisms.octocanum;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class OctocanumFront extends HwServo {
    public OctocanumFront(HardwareMap hardwareMap) throws IllegalArgumentException {
        super(hardwareMap, "octo_front");
    }
    public enum ServoState{
        UP,
        DOWN;
    }

    private OctocanumBack.ServoState servoState = OctocanumBack.ServoState.UP;

    public void engage(){
        servoState = OctocanumBack.ServoState.DOWN;
        setPosition(216/255f);
    }

    public void raise(){
        servoState = OctocanumBack.ServoState.UP;
        setPosition(110/255f);
    }

    public OctocanumBack.ServoState getState(){
        return servoState;
    }
}
