package org.firstinspires.ftc.teamcode.mechanisms.intake;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class Popper extends HwServo {
    public static float NEUTRAL;
    public static float POP;

    public Popper(HardwareMap hwMap) {
        super(hwMap, "popper");
    }

    public void pop() {
        setPosition(POP);
    }

    public void neutral() {
        setPosition(NEUTRAL);
    }
}
