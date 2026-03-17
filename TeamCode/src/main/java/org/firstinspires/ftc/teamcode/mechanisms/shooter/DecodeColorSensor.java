package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.hardware.HwColorSensor;

@Config
public class DecodeColorSensor extends HwColorSensor {
    public static double GREENMIN = 150;
    public static double GREENMAX = 170;
    public static double PURPLEMIN = 200;
    public static double PURPLEMAX = 250;
    public static double DISTANCEMIN = 0;
    public static double DISTANCEMAX = 40;
    public DecodeColorSensor(HardwareMap hardwareMap,String id){
        super(hardwareMap,id);
    }
    @Override
    public ArtifactColor getColor(double hue, double dist) {
        if (objectDetected()){
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
        return PURPLEMIN <= getHue() && PURPLEMAX >= getHue();
    }

    public boolean greenDetected(){
        return GREENMIN <= getHue() && GREENMAX >= getHue();
    }

    public boolean objectDetected(){
        return DISTANCEMIN <= getDistanceMM() && DISTANCEMAX >= getDistanceMM();
    }
}
