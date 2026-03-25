package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerColorSensors;
import org.firstinspires.ftc.teamcode.utils.ArtifactColor;

import java.util.Arrays;

//Eric Debug Thanks
public class IntakeThread {
    public static double COLOR_DETECTION_DELAY = 50;
    public static int COLOR_DETECTION_PERIOD;
    private final SpindexerColorSensors colorSensors;
    private final ArtifactColor[] tableCompartments;
    private final ArtifactColor[] relativeCompartments;
    private final IntakeDistance intakeDistance;
    private int hypotheticalNumBalls = 0;

    public IntakeThread(ArtifactColor[] tableCompartments, SpindexerColorSensors colorManager, IntakeDistance intakeDistance) {
        this.colorSensors = colorManager;
        this.tableCompartments = tableCompartments;
        this.intakeDistance = intakeDistance;
        relativeCompartments = new ArtifactColor[3];
    }

    public void update() {
        if (hypotheticalNumBalls < 2) {
            if (intakeDistance.artifactPassedThrough()) {
                hypotheticalNumBalls = 2;
                intakeDistance.setDetectionState(IntakeDistance.DetectionState.COMPARTMENT_THREE);
                Scheduler.getInstance().schedule(
                        new Sequential(
                                new Wait(COLOR_DETECTION_DELAY),
                                new Race(
                                        new Command()
                                                .setExecute(this::updateInternalColors)
                                                .setDone(() -> getNumBalls() == 2),
                                        new Sequential(
                                                new Wait(COLOR_DETECTION_PERIOD),
                                                new Instant(() -> {
                                                    hypotheticalNumBalls--;
                                                    start();
                                                })
                                        ),
                                        new WaitUntil(() -> hypotheticalNumBalls != 3)
                                )
                        )
                );
            }
        } else if (hypotheticalNumBalls == 2) {
            if (intakeDistance.hasArtifact()) {
                hypotheticalNumBalls = 3;
                intakeDistance.setDetectionState(IntakeDistance.DetectionState.NONE);
                colorSensors.expressThree();
            }
        }
    }

    public void updateColors(int currentIndex) {
        colorSensors.updateColors();
        for (int i = 0; i < tableCompartments.length; i++)
            tableCompartments[i] = colorSensors.getColor(i, currentIndex);
    }

    private void updateInternalColors() {
        colorSensors.updateColors();
        for (int i = 0; i < relativeCompartments.length; i++)
            relativeCompartments[i] = colorSensors.getColor(i);
    }

    public int getNumBalls() {
        if (hypotheticalNumBalls == 3) return 3;
        int balls = 0;
        for (ArtifactColor color : relativeCompartments)
            if (!color.equals(ArtifactColor.NONE))
                balls++;
        return balls;
    }

    public void reset() {
        hypotheticalNumBalls = 0;
        Arrays.fill(relativeCompartments, ArtifactColor.NONE);
    }

    public void start() {
        intakeDistance.setDetectionState(IntakeDistance.DetectionState.PASS_THROUGH_1);
    }
}
