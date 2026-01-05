package org.firstinspires.ftc.teamcode.utils.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Function;

public class HwDistance extends HwSensor<Double, DistanceSensor> {
    private Function<Double, Double> readingToDistance;

    public HwDistance(HardwareMap hardwareMap, String id) {
        super(hardwareMap, id, DistanceSensor.class);
    }

    public HwDistance sensorConversion(Function<Double, Double> conversion) {
        this.readingToDistance = conversion;
        return this;
    }

    @Override
    public Double get() {
        return readingToDistance.apply(sensor.getDistance(DistanceUnit.INCH));
    }

    public double getDistance(DistanceUnit distanceUnit) {
        return distanceUnit.fromInches(getReading());
    }

    public double getDistance() {
        return getReading();
    }
}
