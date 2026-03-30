package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.ArtifactColor;

import java.util.List;

public class SpindexerColorSensors {
    private final DecodeColorSensor leftColorSensor;
    private final DecodeColorSensor rightColorSensor;
    private List<ArtifactColor> compartmentColors;

    public SpindexerColorSensors(HardwareMap hardwareMap){
        leftColorSensor = new DecodeColorSensor(hardwareMap, "colorLeft");
        rightColorSensor = new DecodeColorSensor(hardwareMap, "colorRight");
        compartmentColors = List.of(ArtifactColor.NONE, ArtifactColor.NONE, ArtifactColor.NONE);
    }

    public void updateColors(){
        /*
        ArtifactColor middleColor = List.of(leftColorSensor.getColor(), rightColorSensor.getColor()).contains(ArtifactColor.GREEN) ?
                ArtifactColor.PURPLE : ArtifactColor.GREEN;
        compartmentColors = List.of(middleColor, leftColorSensor.getColor(), rightColorSensor.getColor());
         */
    }

    public List<ArtifactColor> getCompartmentColors(){
        return compartmentColors;
    }

    public ArtifactColor getColor(int index, int currentIndex) {
        return ArtifactColor.NONE;
    }

    public ArtifactColor getColor(int index) {
        return getColor(index, 1);
    }

    public boolean hasColor(int index, int currentIndex) {
        return !getColor(index, currentIndex).equals(ArtifactColor.NONE);
    }

    public void update(){
        leftColorSensor.update();
        rightColorSensor.update();
        updateColors();
    }
}
