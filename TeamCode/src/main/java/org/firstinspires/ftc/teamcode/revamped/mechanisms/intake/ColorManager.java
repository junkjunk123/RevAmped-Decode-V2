package org.firstinspires.ftc.teamcode.revamped.mechanisms.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.revamped.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.revamped.utils.HwColorSensor;

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
    public final HwColorSensor colorThree;

    public ColorManager(HardwareMap hardwareMap) {
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

        colorThree = new HwColorSensor(hardwareMap, "colorThree") {
            @Override
            public ArtifactColor getColor(double hue, double dist) {
                return null;
            }
        };
    }

    public void update() {
        colorOne.update();
        colorTwo.update();
        colorThree.update();
    }

    public ArtifactColor getColorOne() {
        return colorOne.getColor();
    }

    public ArtifactColor getColorTwo() {
        return colorTwo.getColor();
    }

    public ArtifactColor getColorThree() {
        return colorThree.getColor();
    }
}
