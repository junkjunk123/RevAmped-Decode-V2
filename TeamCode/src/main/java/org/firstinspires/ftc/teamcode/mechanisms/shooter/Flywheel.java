package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;
import org.firstinspires.ftc.teamcode.utils.logging.DecodeLogger;

@Config
public class Flywheel extends HwMotor {
    private static final long TRACKING_LOG_PERIOD_NANOS = 150_000_000L;
    private static final double TRACKING_LOG_DELTA = 30.0;
    public static double RADIUS;
    public static int COUNTS_PER_REVOLUTION;
    private final static double ETA = 0.49;

    public static double FAR_VELOCITY;
    public static double MEDIUM_VELOCITY;
    public static double NEAR_VELOCITY;
    public static double CLOSE_AUTO_VELOCITY;
    public static double CORNER_VELOCITY;
    public static double UNSORTED_AUTO_VELOCITY;
    public static double MAX_VELOCITY = 1400;

    private double targetVelocity;
    private final FlywheelController controller;
    private boolean atSpeedLogged;
    private double lastLoggedTargetVelocity = Double.NaN;
    private FlywheelState lastLoggedState;
    private long lastTargetLogNanos;

    public enum FlywheelState {
        FAR,
        MEDIUM,
        NEAR,
        TRACKING,
        CORNER,
        STOPPED,
        NO_PID
    }
    private FlywheelState state = FlywheelState.STOPPED;

    public Flywheel(HardwareMap hardwareMap) {
        super(hardwareMap, "flywheel_right", "flywheel_left");
        setEncoder(Encoder.fromMotor(get()).reverse());
        resetPosition();
        hardware[0].setDirection(DcMotorSimple.Direction.REVERSE);
        hardware[1].setDirection(DcMotorSimple.Direction.FORWARD);
        controller = new FlywheelController();
    }

    public void update() {
        super.update();

        if (isRunning() && state != FlywheelState.NO_PID) {
            double power = controller.update(getVelocityImperial(), targetVelocity);
            setPower(power);
        }

        if (isRunning() && !atSpeedLogged && canShoot()) {
            DecodeLogger.get().info("flywheel", "FLYWHEEL_AT_SPEED",
                    "vel", getVelocityImperial(),
                    "target", targetVelocity,
                    "error", Math.abs(targetVelocity - getVelocityImperial()));
            atSpeedLogged = true;
        } else if (!canShoot()) {
            atSpeedLogged = false;
        }
    };

    private void runToVel(double target) {
        if (Math.abs(targetVelocity - target) > 1.0) {
            resetController();
            atSpeedLogged = false;
        }
        targetVelocity = target;
    }

    public void setVelocity(double target) {
        setStateVelocity(target, FlywheelState.TRACKING, "TRACKING");
    }

    public void medium() {
        setStateVelocity(MEDIUM_VELOCITY, FlywheelState.MEDIUM, "MEDIUM");
    }

    public void far() {
        setStateVelocity(FAR_VELOCITY, FlywheelState.FAR, "FAR");
    }

    public void near() {
        setStateVelocity(NEAR_VELOCITY, FlywheelState.NEAR, "NEAR");
    }

    public void corner() {
        runToVel(CORNER_VELOCITY);
        state = FlywheelState.CORNER;
    }

    public void closeAuto() {
        setStateVelocity(CLOSE_AUTO_VELOCITY, FlywheelState.TRACKING, "CLOSE_AUTO");
    }

    public void unsortedAuto() {
        setStateVelocity(UNSORTED_AUTO_VELOCITY, FlywheelState.TRACKING, "UNSORTED_AUTO");
    }

    public void stop() {
        state = FlywheelState.STOPPED;
        atSpeedLogged = false;
        logTargetIfNeeded(0.0, FlywheelState.STOPPED.name());
        setPower(0);
    }

    public void runAtPower(double power) {
        boolean changed = state != FlywheelState.NO_PID || Math.abs(getPower() - power) > 0.01;
        state = FlywheelState.NO_PID;
        atSpeedLogged = false;
        setPower(power);
        if (changed) {
            DecodeLogger.get().info("flywheel", "FLYWHEEL_TARGET_SET",
                    "targetVel", 0.0,
                    "targetPower", power,
                    "mode", FlywheelState.NO_PID.name());
            lastLoggedState = state;
            lastLoggedTargetVelocity = 0.0;
            lastTargetLogNanos = System.nanoTime();
        }
    }

    private void resetController() {
        controller.reset(getVelocityImperial());
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public static double ticksPerInch() {
        return (double) COUNTS_PER_REVOLUTION / (2 * Math.PI * RADIUS);
    }

    public double getVelocityImperial() {
        return getVelocity() / ticksPerInch();
    }

    public boolean canShoot() {
        return Math.abs(targetVelocity - getVelocityImperial()) < targetVelocity / 40;
    }

    public boolean atState(FlywheelState state) {
        return state == this.state;
    }

    public String getState() {
        return state.name();
    }

    public double getError() {
        return controller.getError();
    }

    public double getLaunchVelocity() {
        return targetVelocity / RADIUS * 1.417 * ETA;
    }

    public boolean isStopped() {return state == FlywheelState.STOPPED;}

    public boolean isRunning() {return !isStopped();}

    @Override
    public void deenergize() {
        super.deenergize();
        state = FlywheelState.STOPPED;
    }

    /**
     * @return inches/sec
     */
    public double getFilteredVelocity() {
        return controller.getFilteredVelocity();
    }

    public FlywheelController getController() {
        return controller;
    }

    private void setStateVelocity(double target, FlywheelState newState, String mode) {
        runToVel(target);
        state = newState;
        logTargetIfNeeded(target, mode);
    }

    private void logTargetIfNeeded(double target, String mode) {
        boolean stateChanged = state != lastLoggedState;
        double deltaThreshold = state == FlywheelState.TRACKING ? TRACKING_LOG_DELTA : 1.0;
        boolean targetChanged = Double.isNaN(lastLoggedTargetVelocity)
                || Math.abs(target - lastLoggedTargetVelocity) >= deltaThreshold;

        long nowNanos = System.nanoTime();
        if (!stateChanged && !targetChanged) return;
        if (!stateChanged && state == FlywheelState.TRACKING
                && nowNanos - lastTargetLogNanos < TRACKING_LOG_PERIOD_NANOS) return;

        DecodeLogger.get().info("flywheel", "FLYWHEEL_TARGET_SET",
                "targetVel", target,
                "mode", mode);
        lastLoggedState = state;
        lastLoggedTargetVelocity = target;
        lastTargetLogNanos = nowNanos;
    }
}
