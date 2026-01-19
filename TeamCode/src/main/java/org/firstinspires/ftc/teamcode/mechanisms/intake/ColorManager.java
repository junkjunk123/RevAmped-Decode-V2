package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.hardware.HwColorSensor;

import java.util.List;

public class ColorManager {
    public static float minPurpleHueZero = 270;
    public static float maxPurpleHueZero = 290;
    public static float maxPurpleDistanceZero = 100;
    public static float minPurpleHueOne = 270;
    public static float maxPurpleHueOne = 290;
    public static float maxPurpleDistanceOne = 100;
    public static float minPurpleHueTwo = 270;
    public static float maxPurpleHueTwo = 290;
    public static float maxPurpleDistanceTwo = 100;

    public static float minGreenHueZero = 100;
    public static float maxGreenHueZero = 140;
    public static float maxGreenDistanceZero = 100;
    public static float minGreenHueOne = 100;
    public static float maxGreenHueOne = 140;
    public static float maxGreenDistanceOne = 100;
    public static float minGreenHueTwo = 100;
    public static float maxGreenHueTwo = 140;
    public static float maxGreenDistanceTwo = 100;

    public final HwColorSensor colorOne;
    public final HwColorSensor colorTwo;
    public final HwColorSensor colorZero;
    public final List<HwColorSensor> allSensors;

    public ColorManager(HardwareMap hardwareMap) {
        colorOne = null;
        colorTwo = null;
        colorZero = null;
        /*
        colorOne = new HwColorSensor(hardwareMap, "colorOne") {
            @Override
            public ArtifactColor getColor(double hue, double dist) {
                return null;
            }
        };

        colorTwo = new HwColorSensor(hardwareMap, "colorTwo") {
            @Override
            public ArtifactColor getColor(double hue, double dist) {
                return null;
            }
        };

        colorZero = new HwColorSensor(hardwareMap, "colorZero") {
            @Override
            public ArtifactColor getColor(double hue, double dist) {
                return null;
            }
        };

         */

        allSensors = List.of();
        //allSensors = List.of(colorZero, colorOne, colorTwo);
    }

    public void update() {
        for (HwColorSensor color : allSensors)
            color.update();
    }

    public ArtifactColor getColorOne() {
        return colorOne.getColor();
    }

    public ArtifactColor getColorTwo() {
        return colorTwo.getColor();
    }

    public ArtifactColor getColorZero() {
        return colorZero.getColor();
    }

    public ArtifactColor getColor(int sensor) {
        return allSensors.get(sensor).getColor();
    }
}
