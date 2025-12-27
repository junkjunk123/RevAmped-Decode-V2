package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class Hood extends HwServo {
    public static float HOOD_MIN_RAD;
    public static float HOOD_MAX_RAD;
    public static float HOOD_MIN_POS;
    public static float HOOD_MAX_POS;

    public static float REST;
    public static float NEAR_PRESET;
    public static float FAR_PRESET;
    public static float MEDIUM_PRESET;

    public Hood(HardwareMap hwMap) {
        super(hwMap, "hood");
    }

    public void rest() {
        setPosition(REST);
    }

    public void near() {
        setPosition(NEAR_PRESET);
    }

    public void far() {
        setPosition(FAR_PRESET);
    }

    public void medium() {
        setPosition(MEDIUM_PRESET);
    }
}
