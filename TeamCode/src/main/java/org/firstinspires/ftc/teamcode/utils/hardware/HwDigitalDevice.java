package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HwDigitalDevice implements HwDevice {
    private final DigitalChannel sensor;
    private Boolean reading;

    public HwDigitalDevice(HardwareMap hardwareMap, String id) {
        sensor = HwDevice.init(hardwareMap, DigitalChannel.class, id);
    }

    public boolean state() {
        return reading == null ? reading = sensor.getState() : reading;
    }

    public void update() {
        reading = null;
    }
}
