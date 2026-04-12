package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commands.ArtifactColor;

public class SpindexerColorSensors {
    public final DecodeColorSensor leftColorSensor;
    public final DecodeColorSensor rightColorSensor;
    private final ArtifactColor[] compartmentColors;

    public SpindexerColorSensors(HardwareMap hardwareMap){
        leftColorSensor = new DecodeColorSensor(hardwareMap, "colorLeft");
        rightColorSensor = new DecodeColorSensor(hardwareMap, "colorRight");
        compartmentColors = new ArtifactColor[] {ArtifactColor.NONE, ArtifactColor.NONE, ArtifactColor.NONE};
    }

    public void updateColors() {
        if (compartmentColors[2].equals(ArtifactColor.NONE)) {
            compartmentColors[2] = rightColorSensor.getColor();
            return;
        }

        if (compartmentColors[1].equals(ArtifactColor.NONE)) compartmentColors[1] = leftColorSensor.getColor();
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

    public void close() {
        leftColorSensor.close();
        rightColorSensor.close();
    }

    public void update() {
        leftColorSensor.update();
        rightColorSensor.update();
    }
}
