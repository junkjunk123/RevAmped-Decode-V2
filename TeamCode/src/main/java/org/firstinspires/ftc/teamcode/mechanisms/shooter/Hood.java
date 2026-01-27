package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

@Config
public class Hood extends HwServo {
    public static float HOOD_MIN_RAD;
    public static float HOOD_MAX_RAD;
    public static float HOOD_MIN_POS;
    public static float HOOD_MAX_POS;

    public static float REST;
    public static float NEAR_PRESET;
    public static float FAR_PRESET;
    public static float MEDIUM_PRESET;

    public enum HoodState {
        REST,
        NEAR,
        FAR,
        MEDIUM,
        TRACKING
    }
    private HoodState state = HoodState.REST;

    public Hood(HardwareMap hwMap) {
        super(hwMap, "hood");
        rest();
    }

    public void rest() {
        setPosition(REST);
        state = HoodState.REST;
    }

    public void near() {
        setPosition(NEAR_PRESET);
        state = HoodState.NEAR;
    }

    public void far() {
        setPosition(FAR_PRESET);
        state = HoodState.FAR;
    }

    public void medium() {
        setPosition(MEDIUM_PRESET);
        state = HoodState.MEDIUM;
    }

    public void updateTracking(double pos) {
        setPosition(pos);
        state = HoodState.TRACKING;
    }

    public boolean atState(HoodState state) {
        return state == this.state;
    }

    public String getState() {
        return state.name();
    }
}
