package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HwDigitalDevice implements HwDevice {
    private final DigitalChannel sensor;
    private Boolean reading;
    private boolean isFlipped;

    public HwDigitalDevice(HardwareMap hardwareMap, String id) {
        sensor = HwDevice.init(hardwareMap, DigitalChannel.class, id);
    }

    public void setFlipped(boolean flipped) {
        isFlipped = flipped;
    }

    public boolean state() {
        if (reading == null) reading = sensor.getState();
        return reading == !isFlipped;
    }

    public void update() {
        reading = null;
    }
}
