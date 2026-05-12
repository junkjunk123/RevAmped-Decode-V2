package org.firstinspires.ftc.teamcode.mechanisms.intake;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.data.BooleanPeriod;
import org.firstinspires.ftc.teamcode.utils.hardware.HwDigitalDevice;

public class IntakeArtifactDetector extends HwDigitalDevice {
    private final BooleanPeriod period;
    private boolean on = false;
    public static double detectionPeriod;

    public IntakeArtifactDetector(HardwareMap hardwareMap, String id) {
        super(hardwareMap, id);
        this.period = new BooleanPeriod(this::get, detectionPeriod);
    }

    public IntakeArtifactDetector(HardwareMap hardwareMap, String id, int numMisfires) {
        super(hardwareMap, id);
        this.period = new BooleanPeriod(this::get, detectionPeriod, numMisfires);
    }

    @Override
    public void update() {
        super.update();
        if (on) period.update();
    }

    public void start() {
        on = true;
        period.start();
    }

    public void stop() {
        on = false;
    }

    public boolean hasArtifact() {
        if (!on) return false;
        return period.getAsBoolean();
    }

    @Override
    public Boolean getReading() {
        if (!on) return false;
        return super.getReading();
    }

    public boolean isOn() {
        return on;
    }
}