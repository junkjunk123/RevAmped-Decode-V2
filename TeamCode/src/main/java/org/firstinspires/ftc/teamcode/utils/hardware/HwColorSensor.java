package org.firstinspires.ftc.teamcode.utils.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.commands.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.math.calc.HSVValue;
import org.firstinspires.ftc.teamcode.utils.math.calc.RGBValue;

public abstract class HwColorSensor implements HwDevice {
    private ArtifactColor color;
    private final RevColorSensorV3 sensor;
    private RGBValue rgbValues;
    private HSVValue hsvValues;
    private Double distance;
    private Float alpha;
    private final String id;

    public HwColorSensor(HardwareMap hardwareMap, String id) {
        this.id = id;
        sensor = HwDevice.init(hardwareMap, RevColorSensorV3.class, id);
    }

    public abstract ArtifactColor getColor(double hue, double dist);

    public ArtifactColor getColor() {
        return color == null ? color = getColor(getHSVValues().getH(), getDistanceMM()) : color;
    }

    public float getNormalizedAlpha() {
        if (alpha == null) {
            updateRGBValues();
        }

        return alpha;
    }

    protected void updateRGBValues() {
        NormalizedRGBA values = ((NormalizedColorSensor) (sensor)).getNormalizedColors();
        rgbValues = new RGBValue(values.red * 255.0, values.green * 255.0, values.blue * 255.0);
        alpha = values.alpha;
    }

    protected void updateHSVValues() {
        if (rgbValues == null) {
            updateRGBValues();
        }

        hsvValues = rgbValues.toHSV();
    }

    public void update() {
        rgbValues = null;
        hsvValues = null;
        alpha = null;
        distance = null;
        color = null;
    }

    public HSVValue getHSVValues() {
        if (hsvValues == null) {
            updateHSVValues();
        }

        return hsvValues;
    }

    public RGBValue getRGBValues() {
        if (rgbValues == null) {
            updateRGBValues();
        }

        return rgbValues;
    }

    public double getDistanceMM() {
        return distance == null? distance = sensor.getDistance(DistanceUnit.MM) : distance;
    }

    public void setBrandBrushlands() {
        ((LynxI2cDeviceSynch) sensor.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
    }

    public RevColorSensorV3 getSensor() {
        return sensor;
    }

    public void close() {
        sensor.close();
    }

    @NonNull
    @Override
    public String toString() {
        return id;
    }
}
