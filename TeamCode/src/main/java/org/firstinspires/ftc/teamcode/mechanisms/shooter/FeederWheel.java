package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;

@Config
public class FeederWheel extends HwMotor {
    public static double kS = 0.003;
    public static double kV = 0.000387;
    public static double P = 0.0012;
    public static double TARGET_VEL;
    public static double SHOOT_VELOCITY;
    public static double INTAKE_VELOCITY;
    public static double INTAKE_NO_SENSORS;
    public static double SHOOT_FAR;
    private double targetVelocity;

    private enum FeederState {
        STOPPED,
        RUNNING
    }

    private FeederState state = FeederState.STOPPED;

    public FeederWheel(HardwareMap hardwareMap) {
        super(hardwareMap, "feeder");
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
        if (targetVelocity > 10) state = FeederState.RUNNING;
        else stop();
    }

    public void start() {
        setTargetVelocity(TARGET_VEL);
    }

    public void intake() {
        setTargetVelocity(INTAKE_VELOCITY);
    }

    public void shoot(){setTargetVelocity(SHOOT_VELOCITY);}

    public void intakeSlow(){setTargetVelocity(INTAKE_NO_SENSORS);}

    public void shootFar(){setTargetVelocity(SHOOT_FAR);}

    public void stop() {
        targetVelocity = 0;
        state = FeederState.STOPPED;
        setPower(0);
    }

    public void update() {
        if (state == FeederState.RUNNING) setPower(calculatePID());
    }

    public double calculatePID() {
        double error = targetVelocity - getVelocity();
        return kS + kV * targetVelocity + P * error;
    }
}
