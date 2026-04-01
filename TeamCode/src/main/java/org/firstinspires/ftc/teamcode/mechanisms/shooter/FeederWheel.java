package org.firstinspires.ftc.teamcode.mechanisms.shooter;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;

@Config
public class FeederWheel extends HwMotor {
    public static double kS = 0.05;
    public static double kV = 0.00039;
    public static double P = 0.00045;
    public static double TARGET_VEL;

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
