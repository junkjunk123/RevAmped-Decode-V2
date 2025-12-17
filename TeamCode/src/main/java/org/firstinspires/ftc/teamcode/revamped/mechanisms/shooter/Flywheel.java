package org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.revamped.math.calc.Extrapolator;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.HwMotor;

public class Flywheel extends HwMotor {
    public static double P;
    public static double K_static;
    public static double K_V;
    public static double STATE_STDDEV = 3;
    public static double MEASUREMENT_STDDEV = 0.4;
    public static double FLYWHEEL_FAR_VELOCITY;
    public static double FLYWHEEL_NEAR_VELOCITY;
    public static double FLYWHEEL_MEDIUM_VELOCITY;
    public static double RADIUS; //Inches
    public static int COUNTS_PER_REVOLUTION;

    private double targetVelocity;
    private boolean running;
    private final Extrapolator velocityProjector;
    private final KalmanFilter filter;

    public Flywheel(HardwareMap hardwareMap) {
        super(hardwareMap, "flywheel_right", "flywheel_left");
        filter = new KalmanFilter(new KalmanFilterParameters(STATE_STDDEV, MEASUREMENT_STDDEV));
        velocityProjector = new Extrapolator(2);
    }

    public void update() {
        super.update();

        if (running) {
            double velocity = getVelocity();
            double velTarget = targetVelocity * ticksPerInch();
            filter.update(velocity, velocityProjector.extrapolate().orElse(velocity));
            velocityProjector.update(velocity);
            velocity = filter.getState();
            double error = velTarget - velocity;
            double controlOutput = P * error + K_V * velTarget + K_static * Math.signum(error);
            setPower(controlOutput);
        }
    }

    public void setTargetVelocity(double target) {
        if (Math.abs(targetVelocity - target) > 1)
            resetController();

        targetVelocity = target;

        if (targetVelocity != 0)
            running = true;
    }

    public void stop() {
        running = false;
        setPower(0);
    }

    private void resetController() {
        filter.reset(getVelocity(), 1,1);
        velocityProjector.reset();
    }

    public static double ticksPerInch() {
        return (double) COUNTS_PER_REVOLUTION / 2 / Math.PI / RADIUS;
    }

    public double getVelocityImperial() {
        return getVelocity() / ticksPerInch();
    }

    public boolean isRunning() {
        return running;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }
}
