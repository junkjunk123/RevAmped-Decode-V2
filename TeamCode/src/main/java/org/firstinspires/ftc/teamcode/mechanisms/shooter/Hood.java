package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;
import org.firstinspires.ftc.teamcode.utils.logging.DecodeLogger;

@Config
public class Hood extends HwServo {
    private static final long TRACKING_LOG_PERIOD_NANOS = 150_000_000L;
    private static final double TRACKING_LOG_DELTA = 0.02;
    public static float HOOD_MIN_RAD;
    public static float HOOD_MAX_RAD;
    public static float HOOD_MIN_POS;
    public static float HOOD_MAX_POS;

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
    private double lastTrackingLogPos = Double.NaN;
    private long lastTrackingLogNanos;

    public Hood(HardwareMap hwMap) {
        super(hwMap, "hood");
        rest();
    }

    public void rest() {
        setState(REST, HoodState.REST);
    }

    public void near() {
        setState(NEAR_PRESET, HoodState.NEAR);
    }

    public void far() {
        setState(FAR_PRESET, HoodState.FAR);
    }

    public void corner() {
        setPosition(CORNER_PRESET);
        state = HoodState.CORNER;
    }


    public void medium() {
        setState(MEDIUM_PRESET, HoodState.MEDIUM);
    }

    public void unsortedAuto() {
        setState(UNSORTED_AUTO, HoodState.TRACKING);
    }

    public void updateTracking(double pos) {
        boolean moved = setPosition(pos);
        boolean stateChanged = state != HoodState.TRACKING;
        state = HoodState.TRACKING;
        if (shouldLogTracking(pos, moved, stateChanged)) {
            DecodeLogger.get().debug("hood", "HOOD_STATE_SET", "state", state.name(), "pos", pos);
        }
    }

    public boolean atState(HoodState state) {
        return state == this.state;
    }

    public String getState() {
        return state.name();
    }
    public HoodState getCurrentState(){return state;}

    private void setState(double targetPosition, HoodState nextState) {
        boolean moved = setPosition(targetPosition);
        boolean stateChanged = state != nextState;
        state = nextState;
        if (moved || stateChanged) {
            DecodeLogger.get().info("hood", "HOOD_STATE_SET", "state", nextState.name());
        }
    }

    private boolean shouldLogTracking(double pos, boolean moved, boolean stateChanged) {
        if (stateChanged) {
            lastTrackingLogPos = pos;
            lastTrackingLogNanos = System.nanoTime();
            return true;
        }
        if (!moved) return false;

        long nowNanos = System.nanoTime();
        boolean movedEnough = Double.isNaN(lastTrackingLogPos)
                || Math.abs(pos - lastTrackingLogPos) >= TRACKING_LOG_DELTA;
        if (!movedEnough) return false;
        if (nowNanos - lastTrackingLogNanos < TRACKING_LOG_PERIOD_NANOS) return false;

        lastTrackingLogPos = pos;
        lastTrackingLogNanos = nowNanos;
        return true;
    }
}
