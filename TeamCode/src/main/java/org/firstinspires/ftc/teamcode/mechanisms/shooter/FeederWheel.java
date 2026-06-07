package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;

@Config
public class FeederWheel extends HwMotor {
    public static double kS = 0.001;
    public static double kV = 0.00037;
    public static double P = 0.0007;
    public static double TARGET_VEL;

    public static double INTAKE_POWER;
    public static double INTAKE_NO_SENSORS;
    public static double TRANSFER_FAR;


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
        setPower(INTAKE_POWER);
    }

    public void intakeSlow()  {setPower(INTAKE_NO_SENSORS);}

    public void transferFar(){setPower(TRANSFER_FAR);}

    public void stop() {
        targetVelocity = 0;
        state = FeederState.STOPPED;
        setPower(0);
    }

    public void update() {
        if (state == FeederState.RUNNING) setPower(1.0); //setPower(calculatePID());
    }

    public double calculatePID() {
        double error = targetVelocity - getVelocity();
        return kS + kV * targetVelocity + P * error;
    }
}
