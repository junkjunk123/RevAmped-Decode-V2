package org.firstinspires.ftc.teamcode.mechanisms.intake;

import org.firstinspires.ftc.teamcode.utils.ArtifactColor;

public class IntakeThread {
    private final ColorManager colorSensors;
    private final ArtifactColor[] tableCompartments;

    public IntakeThread(ArtifactColor[] tableCompartments, ColorManager colorManager) {
        this.colorSensors = colorManager;
        this.tableCompartments = tableCompartments;
    }

    public void update() {}

    public void updateColors() {
        for (int i = 0; i < tableCompartments.length; i++)
            tableCompartments[i] = colorSensors.getColor(i);
    }
}
