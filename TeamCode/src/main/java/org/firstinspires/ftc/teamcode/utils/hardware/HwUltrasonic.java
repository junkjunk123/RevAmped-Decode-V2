package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.Function;

public class HwUltrasonic extends HwSensor<Double, AnalogInput> {
    private Function<Double, Double> voltageToDistance;

    public HwUltrasonic(HardwareMap hardwareMap, String id) {
        super(hardwareMap, id, AnalogInput.class);
    }

    public HwUltrasonic sensorConversion(Function<Double, Double> conversion) {
        this.voltageToDistance = conversion;
        return this;
    }

    @Override
    public Double get() {
        return voltageToDistance.apply(sensor.getVoltage());
    }

    public double getDistance() {
        return getReading();
    }
}
