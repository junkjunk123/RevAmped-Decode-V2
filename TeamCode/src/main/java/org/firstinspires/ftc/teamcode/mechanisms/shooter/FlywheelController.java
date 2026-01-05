package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.KalmanFilterParameters;

import org.firstinspires.ftc.teamcode.math.calc.SingleStateKalman;

/**
 * Triple-mode hybrid flywheel controller using enum for mode,
 * with SPINUP hysteresis, REGEN hysteresis, and HOLD mode.
 */
@Config
public class FlywheelController {
    public enum Mode {
        SPINUP,
        REGEN,
        HOLD
    }

    // --- PSF gains ---
    public static double GAIN = 0.0006;
    public static double VELOCITY_FEEDFORWARD = 0.00072;
    public static double SECONDARY_VEL_FEEDFORWARD = 0.0008;
    public static double SECONDARY_SWITCH = 800;
    public static double STATIC_FEEDFORWARD = 0.055;

    // --- Spin-up settings ---
    public static double KICK_POWER = 1.0;
    public static double SPINUP_RATIO = 0.75;
    public static double SPINUP_HYSTERESIS = 0.7;

    // --- Regeneration settings ---
    public static double REGEN_RATIO = 0.90;
    public static double REGEN_HYSTERESIS = 0.85;
    public static double REGEN_KICK = 0.2;
    public static double REGEN_GAIN = 0.0003;

    // --- Kalman Filter ---
    public static double STATE_STANDARD_DEVIATION = 2;
    public static double MEASUREMENT_STANDARD_DEVIATION = 0.2;

    private final SingleStateKalman filter;
    private double targetVelocity = 0.0;
    private double error = 0;
    private Mode mode = Mode.SPINUP;

    public FlywheelController() {
        filter = new SingleStateKalman(new KalmanFilterParameters(STATE_STANDARD_DEVIATION, MEASUREMENT_STANDARD_DEVIATION));
    }

    public double update(double measuredVelocity, double commandedVelocity) {
        // --- update Kalman filter ---
        filter.update(measuredVelocity);
        double filteredVelocity = filter.getState();
        targetVelocity = commandedVelocity;

        // --- compute error ---
        error = targetVelocity - filteredVelocity;

        // --- Mode selection with hysteresis ---
        switch (mode) {
            case SPINUP:
                if (filteredVelocity > SPINUP_HYSTERESIS * targetVelocity) {
                    // exit SPINUP → REGEN or HOLD
                    mode = (filteredVelocity < REGEN_RATIO * targetVelocity) ? Mode.REGEN : Mode.HOLD;
                }
                break;
            case REGEN:
                if (filteredVelocity > REGEN_HYSTERESIS * targetVelocity) {
                    mode = Mode.HOLD;
                } else if (filteredVelocity < SPINUP_RATIO * targetVelocity) {
                    mode = Mode.SPINUP;
                }
                break;
            case HOLD:
                if (filteredVelocity < SPINUP_RATIO * targetVelocity) {
                    mode = Mode.SPINUP;
                } else if (filteredVelocity < REGEN_RATIO * targetVelocity) {
                    mode = Mode.REGEN;
                }
                break;
        }

        // --- Control law based on mode ---
        return switch (mode) {
            case SPINUP -> KICK_POWER * Math.signum(error);
            case REGEN -> {
                double velFF = targetVelocity < SECONDARY_SWITCH ? targetVelocity * SECONDARY_VEL_FEEDFORWARD
                        : targetVelocity * VELOCITY_FEEDFORWARD;
                double staticFF = (STATIC_FEEDFORWARD + REGEN_KICK) * Math.signum(error);
                yield REGEN_GAIN * error + velFF + staticFF;
            }
            default -> {
                double velFF = targetVelocity < SECONDARY_SWITCH ? targetVelocity * SECONDARY_VEL_FEEDFORWARD
                        : targetVelocity * VELOCITY_FEEDFORWARD;
                yield GAIN * error + velFF + STATIC_FEEDFORWARD * Math.signum(error);
            }
        };
    }

    public void reset(double measuredVelocity) {
        filter.reset(measuredVelocity, 1.0);
        error = 0;
        mode = Mode.SPINUP;
    }

    public Mode getMode() {
        return mode;
    }

    public double getFilteredVelocity() {
        return filter.getState();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getError() {
        return error;
    }
}
