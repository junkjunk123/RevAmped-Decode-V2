package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class HwVoltageSensor {
    private final VoltageSensor voltageSensor;
    private Double voltage;

    public HwVoltageSensor(HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void update() {
        voltage = null;
    }

    public double getVoltage() {
        if (voltage == null) voltage = voltageSensor.getVoltage();
        return voltage;
    }
}
