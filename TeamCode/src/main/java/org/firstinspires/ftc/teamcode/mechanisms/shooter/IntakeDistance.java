package org.firstinspires.ftc.teamcode.mechanisms.shooter;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.BooleanPeriod;
import org.firstinspires.ftc.teamcode.utils.FirstOrderLowPass;
import org.firstinspires.ftc.teamcode.utils.hardware.HwDistance;

public class IntakeDistance extends HwDistance {
    public static double PERIOD_MS;
    public static double MAX_DIST;
    private final FirstOrderLowPass lowPass;
    private final BooleanPeriod timer;

    public IntakeDistance(HardwareMap hardwareMap) {
        super(hardwareMap, "intakeDistance");
        lowPass = new FirstOrderLowPass(0.6);
        timer = new BooleanPeriod(() -> getDistanceFiltered() <= MAX_DIST, PERIOD_MS);
    }

    public boolean hasArtifact() {
        return timer.getAsBoolean();
    }

    @Override
    public void update() {
        timer.update();
    }

    public void start() {
        timer.start();
    }

    public double getDistanceFiltered() {
        return lowPass.update(getDistance());
    }
}
