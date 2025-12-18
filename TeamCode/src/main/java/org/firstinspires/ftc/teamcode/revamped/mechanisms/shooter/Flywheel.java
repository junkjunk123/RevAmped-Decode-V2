package org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter;

import com.pedropathing.control.KalmanFilterParameters;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.revamped.math.calc.SingleStateKalman;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.HwMotor;

public class Flywheel extends HwMotor {
    public static double P;
    public static double kStatic;
    public static double kV;
    public static double kA;

    public static double STATE_STDDEV;
    public static double MEASUREMENT_STDDEV;

    public static double RADIUS;
    public static int COUNTS_PER_REVOLUTION;

    private static double MAX_ACCELERATION;

    private double targetVelocity;
    private double targetAcceleration;
    private boolean running;

    private final SingleStateKalman filter;
    private double lastTime;

    public Flywheel(HardwareMap hardwareMap) {
        super(hardwareMap, "flywheel_right", "flywheel_left");
        filter = new SingleStateKalman(new KalmanFilterParameters(STATE_STDDEV, MEASUREMENT_STDDEV));
    }

    public void update() {
        super.update();

        long now = System.nanoTime();
        double dt;
        if (lastTime == 0) {
            dt = 0.02; // assume 20ms for the first loop
        } else {
            dt = (now - lastTime) * 1e-9;
        }

        lastTime = now;

        if (!running) return;

        double measuredVelocity = getVelocity();
        updateKalman(measuredVelocity);
        updateMotionProfile(dt);
        double control = computeControl(getFilteredVelocity());
        setPower(control);
    };

    public void setTargetVelocity(double target) {
        if (Math.abs(targetVelocity - target) > 1.0)
            resetController();
        targetVelocity = target;
        running = target != 0;
    }

    public void stop() {
        running = false;
        setPower(0);
    }

    private void resetController() {
        filter.reset(getVelocity(), 1.0);
    }

    private void updateKalman(double measuredVelocity) {
        filter.update(measuredVelocity);
    }

    private void updateMotionProfile(double dt) {
        double filteredVelocity = filter.getState();
        double error = targetVelocity - filteredVelocity;
        targetAcceleration = Range.clip(error / dt, -MAX_ACCELERATION, MAX_ACCELERATION);
        targetVelocity = filteredVelocity + targetAcceleration * dt;
    }

    private double computeControl(double filteredVelocity) {
        double error = targetVelocity - filteredVelocity;
        double ff = kStatic * Math.signum(error) + kV * targetVelocity + kA * targetAcceleration;
        return ff + P * error;
    }

    public boolean isRunning() {
        return running;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getFilteredVelocity() {
        return filter.getState();
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
}
