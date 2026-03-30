package org.firstinspires.ftc.teamcode.mechanisms.intake;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.BooleanPeriod;
import org.firstinspires.ftc.teamcode.utils.hardware.HwDigitalDevice;

public class IntakeArtifactDetector extends HwDigitalDevice {
    private final BooleanPeriod period;
    private boolean on = false;

    public IntakeArtifactDetector(HardwareMap hardwareMap, String id, double period) {
        super(hardwareMap, id);
        this.period = new BooleanPeriod(this::get, period);
    }

    public IntakeArtifactDetector(HardwareMap hardwareMap, String id, double period, int numMisfires) {
        super(hardwareMap, id);
        this.period = new BooleanPeriod(this::get, period, numMisfires);
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
        return period.getAsBoolean();
    }

    public boolean isOn() {
        return on;
    }
}