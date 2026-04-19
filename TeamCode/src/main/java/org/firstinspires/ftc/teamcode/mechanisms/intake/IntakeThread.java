package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerColorSensors;
import org.firstinspires.ftc.teamcode.utils.commands.ArtifactColor;

import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

//Eric Debug Thanks
public class IntakeThread {
    public static boolean useSensors;
    public boolean hasThree;
    public static double COLOR_DETECTION_DELAY = 200;
    public static int COLOR_DETECTION_PERIOD = 100;
    public final SpindexerColorSensors colorSensors;
    private final ArtifactColor[] tableCompartments;
    private final ArtifactColor[] relativeCompartments;
    private final IntakeArtifactDetector intakeDistance;
    private int hypotheticalNumBalls = 0;
    private enum DetectionState{
        IDLE,
        WAITINGTODETECT,
        DETECTING

    }

    private DetectionState detectionState = DetectionState.IDLE;

    public IntakeThread(ArtifactColor[] tableCompartments, SpindexerColorSensors colorManager, IntakeArtifactDetector intakeDistance) {
        useSensors = true;
        hasThree = false;
        this.colorSensors = colorManager;
        this.tableCompartments = tableCompartments;
        this.intakeDistance = intakeDistance;
        relativeCompartments = new ArtifactColor[] {ArtifactColor.NONE, ArtifactColor.NONE, ArtifactColor.NONE};
    }

    public void update() {
        if (!useSensors) return;
        if (getNumBalls() == 3 && !hasThree){
            Scheduler.getInstance().schedule(
                    new Sequential(
                        new Wait(100),
                        new Instant(() -> hasThree = true)
                    )
            );
        }
        if (intakeDistance.hasArtifact()){
            hypotheticalNumBalls = 1;
            intakeDistance.stop();
            Scheduler.getInstance().schedule(
                new Sequential(
                        new Instant(() -> detectionState = DetectionState.WAITINGTODETECT),
                        new Wait(COLOR_DETECTION_DELAY),
                        new Instant(() -> detectionState = DetectionState.DETECTING),
                        new Race(
                                new Command()
                                        .setExecute(this::updateInternalColors)
                                        .setDone(() -> getNumBalls() > hypotheticalNumBalls)
                                        .setEnd(c -> hypotheticalNumBalls = getNumBalls()),
                                new Sequential(
                                        new Wait(COLOR_DETECTION_PERIOD),
                                        new Instant(this::start)
                                ),
                                new Sequential(
                                    new WaitUntil(() -> hypotheticalNumBalls == 3)
                                )
                        ),
                        new Instant(() -> detectionState = DetectionState.IDLE)
                )
        );

        }
//        if (!useSensors) return;
//        if (hypotheticalNumBalls < 2) {
//            if (intakeDistance.hasArtifact()) {
//                hypotheticalNumBalls = 2;
//                intakeDistance.stop();
//                Scheduler.getInstance().schedule(
//                        new Sequential(
//                                new Wait(COLOR_DETECTION_DELAY),
//                                new Race(
//                                        new Command()
//                                                .setExecute(this::updateInternalColors)
//                                                .setDone(() -> getNumBalls() == 2),
//                                        new Sequential(
//                                                new Wait(COLOR_DETECTION_PERIOD),
//                                                new Instant(() -> {
//                                                    hypotheticalNumBalls--;
//                                                    start();
//                                                })
//                                        ),
//                                        new WaitUntil(() -> hypotheticalNumBalls != 3)
//                                )
//                        )
//                );
//            }
//        } else if (hypotheticalNumBalls == 2) {
//            if (!frontDistance.isOn()) frontDistance.start();
//
//            if (frontDistance.hasArtifact()) {
//                hypotheticalNumBalls = 3;
//                expressThree();
//                intakeDistance.stop();
//                frontDistance.stop();
//            }
//        }
    }

    public void updateColors(int currentIndex) {
        colorSensors.updateColors();
        for (int i = 0; i < tableCompartments.length; i++)
            tableCompartments[i] = colorSensors.getColor(i, currentIndex);
        expressThree();
    }

    private void updateInternalColors() {
        colorSensors.updateColors();
        for (int i = 0; i < relativeCompartments.length; i++)
            relativeCompartments[i] = colorSensors.getColor(i);
    }

    public void expressThree() {
        int[] stream = IntStream.range(0, 3)
                .filter(i -> !tableCompartments[i].equals(ArtifactColor.NONE))
                .toArray();
        if (stream.length != 2) return;
        if (tableCompartments[stream[0]].equals(ArtifactColor.NONE) || tableCompartments[stream[1]].equals(ArtifactColor.NONE))
            return;
        ArtifactColor middleColor = List.of(tableCompartments[stream[0]], tableCompartments[stream[1]]).contains(ArtifactColor.GREEN) ?
                ArtifactColor.PURPLE : ArtifactColor.GREEN;

        for (int i = 0; i < 3; i++)
            if (tableCompartments[i].equals(ArtifactColor.NONE)) {
                tableCompartments[i] = middleColor;
                break;
            }
    }

    public int getNumBalls() {
//        if (hypotheticalNumBalls == 3) return 3;
        int balls = 0;
        for (ArtifactColor color : relativeCompartments)
            if (!color.equals(ArtifactColor.NONE))
                balls++;
        return balls+1;
    }

    public DetectionState getDetectionState(){
        return detectionState;
    }

    public int getHypotheticalNumBalls(){
        return hypotheticalNumBalls;
    }

    public ArtifactColor[] getColors(){
        return relativeCompartments;
    }

    public void reset() {
        hypotheticalNumBalls = 0;
        Arrays.fill(relativeCompartments, ArtifactColor.NONE);
        colorSensors.reset();
        intakeDistance.stop();
        hasThree = false;
    }

    public void start() {
        intakeDistance.start();
    }
}
