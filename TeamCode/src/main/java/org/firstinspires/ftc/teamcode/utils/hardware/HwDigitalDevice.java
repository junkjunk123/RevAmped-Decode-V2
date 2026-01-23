package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HwDigitalDevice extends HwSensor<Boolean, DigitalChannel> {
    private boolean isFlipped;

    public HwDigitalDevice(HardwareMap hardwareMap, String id) {
        super(hardwareMap, id, DigitalChannel.class);
    }

    public void setFlipped(boolean flipped) {
        isFlipped = flipped;
    }

    public HwDigitalDevice flip() {
        setFlipped(true);
        return this;
    }

    @Override
    public Boolean get() {
        return sensor.getState() == !isFlipped;
    }

    public Boolean state() {
        return getReading();
    }
}
