package org.firstinspires.ftc.teamcode.mechanisms.intake;

import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.logging.DecodeLogger;

public class IntakeThread {
    private final ColorManager colorSensors;
    private final ArtifactColor[] tableCompartments;
    private boolean noOpWarningLogged;

    public IntakeThread(ArtifactColor[] tableCompartments, ColorManager colorManager) {
        this.colorSensors = colorManager;
        this.tableCompartments = tableCompartments;
    }

    public void update() {
        if (!noOpWarningLogged) {
            DecodeLogger.get().warn("intake", "SUBSYSTEM_DISABLED",
                    "subsystem", "intakeColorPipeline",
                    "reason", "IntakeThread.update has no active implementation");
            noOpWarningLogged = true;
        }
    }

    public void updateColors() {
        for (int i = 0; i < tableCompartments.length; i++)
            tableCompartments[i] = colorSensors.getColor(i);
    }
}
