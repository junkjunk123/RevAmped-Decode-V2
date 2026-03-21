package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

import java.util.stream.IntStream;

public class SpindexerCompartmentManager {
    private final ArtifactColor[] compartments;
    private final SpindexerColorSensors colorSensors;
    private int motifGreenIndex;
    public SpindexerCompartmentManager(SpindexerColorSensors spindexerColorSensors){
        compartments = new ArtifactColor[]{
                ArtifactColor.NONE,
                ArtifactColor.NONE,
                ArtifactColor.NONE
        };

        colorSensors = spindexerColorSensors;
        motifGreenIndex = 0;
    }

    public void updateCompartments(){
        populateCompartments(
                colorSensors.getColor(0),
                colorSensors.getColor(1),
                colorSensors.getColor(2)
        );
    }

    public void populateCompartments(ArtifactColor compartment0, ArtifactColor compartment1, ArtifactColor compartment2){
        compartments[0] = compartment0;
        compartments[1] = compartment1;
        compartments[2] = compartment2;
    }

    public void emptyCompartments(){
        compartments[0] = ArtifactColor.NONE;
        compartments[1] = ArtifactColor.NONE;
        compartments[2] = ArtifactColor.NONE;
    }

    public int[] getGreenIndicies(){
        return IntStream.range(0,3)
                .filter(i -> compartments[i].equals(ArtifactColor.GREEN))
                .toArray();
    }

    public int getGreenIndex(){
        return IntStream.range(0,3)
                .filter(i -> compartments[i].equals(ArtifactColor.GREEN))
                .findFirst()
                .orElse(-1);
    }

    public int getOffset(){
        int offset = getGreenIndex() - motifGreenIndex;
        if(offset%2 == 0 && offset != 0){
            offset = Math.round(Math.copySign(1,-offset));
        }
        return offset;
    }

    public boolean canSort(){
        if (colorSensors.getColor(1) == ArtifactColor.GREEN && colorSensors.getColor(2) == ArtifactColor.GREEN){
            return false;
        }
        return true;
    }

    public void updateMotifIndex(){
        motifGreenIndex = Globals.randomizationState.getGreenIndex();
    }

    public void update(){
        updateCompartments();
    }
}
