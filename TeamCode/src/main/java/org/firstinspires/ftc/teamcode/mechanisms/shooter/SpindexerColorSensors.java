package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.ArtifactColor;

import java.util.List;

public class SpindexerColorSensors {
    private final DecodeColorSensor leftColorSensor;
    private final DecodeColorSensor rightColorSensor;
    private final ArtifactColor[] compartmentColors;

    public SpindexerColorSensors(HardwareMap hardwareMap){
        leftColorSensor = new DecodeColorSensor(hardwareMap, "colorLeft");
        rightColorSensor = new DecodeColorSensor(hardwareMap, "colorRight");
        compartmentColors = new ArtifactColor[] {ArtifactColor.NONE, ArtifactColor.NONE, ArtifactColor.NONE};
    }

    public void updateColors() {
        ArtifactColor left = leftColorSensor.getColor();
        ArtifactColor right = rightColorSensor.getColor();
        if (compartmentColors[1].equals(ArtifactColor.NONE)) compartmentColors[1] = left;
        if (compartmentColors[2].equals(ArtifactColor.NONE)) compartmentColors[2] = right;
    }

    public void expressThree() {
        if (compartmentColors[1].equals(ArtifactColor.NONE) || compartmentColors[2].equals(ArtifactColor.NONE)) return;
        ArtifactColor middleColor = List.of(compartmentColors[1], compartmentColors[2]).contains(ArtifactColor.GREEN) ?
                ArtifactColor.PURPLE : ArtifactColor.GREEN;
        compartmentColors[0] = middleColor;
    }

    public ArtifactColor[] getCompartmentColors(){
        return compartmentColors;
    }

    public ArtifactColor getColor(int index, int currentIndex) {
        return compartmentColors[(index + currentIndex + 3) % 3];
    }

    public ArtifactColor getColor(int index) {
        return getColor(index, 1);
    }

    public boolean hasColor(int index, int currentIndex) {
        return !getColor(index, currentIndex).equals(ArtifactColor.NONE);
    }

    public void update() {
        leftColorSensor.update();
        rightColorSensor.update();
    }
}
