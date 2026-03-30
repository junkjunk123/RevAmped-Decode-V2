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
import java.util.List;

//Eric Debug Thanks
public class IntakeThread {
    public static boolean useSensors;
    public static double COLOR_DETECTION_DELAY = 50;
    public static int COLOR_DETECTION_PERIOD;
    private final SpindexerColorSensors colorSensors;
    private final ArtifactColor[] tableCompartments;
    private final ArtifactColor[] relativeCompartments;
    private final IntakeArtifactDetector intakeDistance;
    private final IntakeArtifactDetector frontDistance;
    private int hypotheticalNumBalls = 0;

    public IntakeThread(ArtifactColor[] tableCompartments, SpindexerColorSensors colorManager, IntakeArtifactDetector intakeDistance, IntakeArtifactDetector frontDistance) {
        this.colorSensors = colorManager;
        this.tableCompartments = tableCompartments;
        this.intakeDistance = intakeDistance;
        this.frontDistance = frontDistance;
        relativeCompartments = new ArtifactColor[3];
    }

    public void update() {
        if (!useSensors) return;
        if (hypotheticalNumBalls < 2) {
            if (intakeDistance.hasArtifact()) {
                hypotheticalNumBalls = 2;
                intakeDistance.stop();
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
            if (!frontDistance.isOn()) frontDistance.start();

            if (frontDistance.hasArtifact()) {
                hypotheticalNumBalls = 3;
                expressThree();
                intakeDistance.stop();
                frontDistance.stop();
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
        if (frontDistance.get()) expressThree();
    }

    public void expressThree() {
        if (tableCompartments[1].equals(ArtifactColor.NONE) || tableCompartments[2].equals(ArtifactColor.NONE)) return;
        ArtifactColor middleColor = List.of(tableCompartments[1], tableCompartments[2]).contains(ArtifactColor.GREEN) ?
                ArtifactColor.PURPLE : ArtifactColor.GREEN;
        tableCompartments[0] = middleColor;
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
        intakeDistance.stop();
        frontDistance.stop();
    }

    public void start() {
        intakeDistance.start();
    }
}
