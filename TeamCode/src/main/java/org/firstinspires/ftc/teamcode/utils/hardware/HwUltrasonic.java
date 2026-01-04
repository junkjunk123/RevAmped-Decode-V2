package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Function;

public class HwUltrasonic extends HwSensor<Double, AnalogInput> {
    private Function<Double, Double> voltageToDistanceInch;

    public HwUltrasonic(HardwareMap hardwareMap, String id) {
        super(hardwareMap, id, AnalogInput.class);
    }

    public HwUltrasonic sensorConversion(Function<Double, Double> conversion) {
        this.voltageToDistanceInch = conversion;
        return this;
    }

    @Override
    public Double get() {
        return voltageToDistanceInch.apply(sensor.getVoltage());
    }

    public double getDistance(DistanceUnit distanceUnit) {
        return distanceUnit.fromInches(getReading());
    }

    public double getDistance() {
        return getReading();
    }
}
