package org.firstinspires.ftc.teamcode.revamped.mechanisms.intake;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.revamped.utils.HwServo;

public class Popper extends HwServo {
    public static float NEUTRAL;
    public static float POP;

    public Popper(HardwareMap hwMap, String id) {
        super(hwMap, id);
    }

    public void pop() {
        setPosition(POP);
    }

    public void neutral() {
        setPosition(NEUTRAL);
    }
}
