package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;

@Config
public class Flywheel extends HwMotor {
    public static double RADIUS;
    public static int COUNTS_PER_REVOLUTION;
    private final static double ETA = 0.49;

    public static double FAR_VELOCITY;
    public static double MEDIUM_VELOCITY;
    public static double NEAR_VELOCITY;
    public static double CLOSE_AUTO_VELOCITY;
    public static double UNSORTED_AUTO_VELOCITY;
    public static double MAX_VELOCITY = 1400;

    private double targetVelocity;
    private final FlywheelController controller;

    public enum FlywheelState {
        FAR,
        MEDIUM,
        NEAR,
        TRACKING,
        STOPPED
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

        if (isRunning()) {
            double power = controller.update(getVelocityImperial(), targetVelocity);
            setPower(power);
        }
    };

    private void runToVel(double target) {
        if (Math.abs(targetVelocity - target) > 1.0)
            resetController();
        targetVelocity = target;
    }

    public void setVelocity(double target) {
        runToVel(target);
        state = FlywheelState.TRACKING;
    }

    public void medium() {
        runToVel(MEDIUM_VELOCITY);
        state = FlywheelState.MEDIUM;
    }

    public void far() {
        runToVel(FAR_VELOCITY);
        state = FlywheelState.FAR;
    }

    public void near() {
        runToVel(NEAR_VELOCITY);
        state = FlywheelState.NEAR;
    }

    public void closeAuto() {
        runToVel(CLOSE_AUTO_VELOCITY);
        state = FlywheelState.TRACKING;
    }

    public void unsortedAuto() {
        runToVel(UNSORTED_AUTO_VELOCITY);
        state = FlywheelState.TRACKING;
    }

    public void stop() {
        state = FlywheelState.STOPPED;
        setPower(0);
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
}