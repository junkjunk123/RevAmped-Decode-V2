package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;
import org.firstinspires.ftc.teamcode.utils.hardware.HwVoltageSensor;
import org.firstinspires.ftc.teamcode.utils.math.ILUT;

@Config
public class Hood extends HwServo {
    public static float HOOD_MIN_RAD;
    public static float HOOD_MAX_RAD;
    public static float HOOD_MIN_POS;
    public static float HOOD_MAX_POS;
    public static float HOOD_FAR_COMP;

    public static float REST;
    public static float CORNER_PRESET;
    public static float NEAR_PRESET;
    public static float FAR_PRESET;
    public static float MEDIUM_PRESET;
    public static float UNSORTED_AUTO;

    public enum HoodState {
        REST,
        NEAR,
        FAR,
        CORNER,
        MEDIUM,
        TRACKING
    }

    private HoodState state = HoodState.REST;
    private double farPos;
    private final HwVoltageSensor voltageSensor;

    public Hood(HardwareMap hwMap, HwVoltageSensor voltageSensor) {
        super(hwMap, "hood");
        this.voltageSensor = voltageSensor;
        rest();
        farPos = FAR_PRESET;
    }

    public Hood(HardwareMap hwMap) {
        this(hwMap, null);
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
        setPosition(farPos);
        state = HoodState.FAR;
    }

    public void corner() {
        setPosition(CORNER_PRESET);
        state = HoodState.CORNER;
    }

    public void medium() {
        setPosition(MEDIUM_PRESET);
        state = HoodState.MEDIUM;
    }

    public void unsortedAuto() {
        setPosition(UNSORTED_AUTO);
        state = HoodState.TRACKING;
    }

    public void updateTracking(double pos) {
        setPosition(pos);
        state = HoodState.TRACKING;
    }

    public void farHoodComp(){
        setPosition(HOOD_FAR_COMP);
    }

    public boolean atState(HoodState state) {
        return state == this.state;
    }

    public void finetuneFar(double ticks) {
        farPos += ticks;
        if (atState(HoodState.FAR)) setPosition(farPos);
    }

    public String getState() {
        return state.name();
    }
}
