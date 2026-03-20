package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.ArtifactColor;

import java.util.List;

public class SpindexerColorSensors {
    private DecodeColorSensor leftColorSensor;
    private DecodeColorSensor rightColorSensor;
    //index 0 -> right ball
    //index 1 -> middle ball
    //index 2 -> left ball
    private List<ArtifactColor> compartmentColors;
    public SpindexerColorSensors(HardwareMap hardwareMap,String leftID,String rightID){
        leftColorSensor = new DecodeColorSensor(hardwareMap,leftID);
        rightColorSensor = new DecodeColorSensor(hardwareMap,rightID);
        compartmentColors = List.of(ArtifactColor.NONE,ArtifactColor.NONE,ArtifactColor.NONE);
    }

    public void updateColors(){
        ArtifactColor middleColor = List.of(leftColorSensor.getColor(),rightColorSensor.getColor()).contains(ArtifactColor.GREEN) ?
                ArtifactColor.PURPLE : ArtifactColor.GREEN;
        compartmentColors = List.of(middleColor,leftColorSensor.getColor(),rightColorSensor.getColor());
    }

    public List<ArtifactColor> getCompartmentColors(){
        return compartmentColors;
    }

    public ArtifactColor getColorAt (int index){return compartmentColors.get(index);}

    public void update(){
        leftColorSensor.update();
        rightColorSensor.update();
        updateColors();
    }
}
