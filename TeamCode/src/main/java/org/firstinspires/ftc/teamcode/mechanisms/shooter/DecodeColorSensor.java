package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commands.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.hardware.HwColorSensor;

@Config
public class DecodeColorSensor extends HwColorSensor {
    public static double GREEN_MIN = 150;
    public static double GREEN_MAX = 170;
    public static double PURPLE_MIN = 200;
    public static double PURPLE_MAX = 250;
    public static double DISTANCE_MAX = 60;

    public DecodeColorSensor(HardwareMap hardwareMap,String id){
        super(hardwareMap,id);
    }

    @Override
    public ArtifactColor getColor(double hue, double dist) {
        if (objectDetected()) {
            if (greenDetected()) return ArtifactColor.GREEN;
            if (purpleDetected()) return ArtifactColor.PURPLE;
        }
        return ArtifactColor.NONE;
    }

    public double getHue(){
        return getHSVValues().getH();
    }

    public double getValue(){
        return getHSVValues().getV();
    }

    public double getSaturation(){
        return getHSVValues().getS();
    }

    public boolean purpleDetected(){
        return PURPLE_MIN <= getHue() && PURPLE_MAX >= getHue();
    }

    public boolean greenDetected(){
        return GREEN_MIN <= getHue() && GREEN_MAX >= getHue();
    }

    public boolean objectDetected() {
        return DISTANCE_MAX >= getDistanceMM();
    }
}
