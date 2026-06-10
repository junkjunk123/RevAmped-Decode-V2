package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;
import org.firstinspires.ftc.teamcode.utils.hardware.HwVoltageSensor;

@Config
public class Flywheel extends HwMotor {
    public static double RADIUS;
    public static int COUNTS_PER_REVOLUTION;
    private final static double ETA = 0.49;

    public static double FAR_VELOCITY;
    public static double MEDIUM_VELOCITY;
    public static double NEAR_VELOCITY;
    public static double CLOSE_AUTO_VELOCITY;
    public static double CORNER_VELOCITY;
    public static double OUTTAKE_POWER;
    public static double CLOSE_PRELOADS_VEL;
    public static double MAX_VELOCITY = 1400;

    private double targetVelocity;
    private final FlywheelController controller;
    private final HwVoltageSensor voltageSensor;

    public enum FlywheelState {
        FAR,
        MEDIUM,
        NEAR,
        TRACKING,
        CORNER,
        STOPPED,
        OUTTAKE,
        NO_PID
    }
    private FlywheelState state = FlywheelState.STOPPED;

    public Flywheel(HardwareMap hardwareMap, HwVoltageSensor voltageSensor) {
        super(hardwareMap, "flywheel_right", "flywheel_left");
        this.voltageSensor = voltageSensor;
        setEncoder(Encoder.fromMotor(get()));
        resetPosition();
        hardware[0].setDirection(DcMotorSimple.Direction.FORWARD);
        hardware[1].setDirection(DcMotorSimple.Direction.REVERSE);
        controller = new FlywheelController();
    }

    public Flywheel(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public void update() {
        super.update();

        if (isRunning() && state != FlywheelState.NO_PID) {
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

    public void corner() {
        runToVel(CORNER_VELOCITY);
        state = FlywheelState.CORNER;
    }

    public void outtake(){
        runAtPower(OUTTAKE_POWER);
        state = FlywheelState.OUTTAKE;
    }

    public void closePreloadsPreset() {
        runToVel(CLOSE_PRELOADS_VEL);
        state = FlywheelState.TRACKING;
    }

    public void stop() {
        state = FlywheelState.STOPPED;
        setPower(0);
    }

    public void runAtPower(double power) {
        state = FlywheelState.NO_PID;
        setPower(power);
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