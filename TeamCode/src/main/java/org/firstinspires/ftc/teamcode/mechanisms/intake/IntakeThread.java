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
    private boolean isEmpty = true;
    public static double COLOR_DETECTION_DELAY = 0;
    public static int COLOR_DETECTION_PERIOD = 1000;
    public final SpindexerColorSensors colorSensors;
    private final ArtifactColor[] tableCompartments;
    private final ArtifactColor[] relativeCompartments;
    private final IntakeArtifactDetector intakeDistance;

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
            useSensors = false;
            Scheduler.getInstance().schedule(
                    new Sequential(
                        new Wait(100),
                        new Instant(() -> {hasThree = true; useSensors = true;})
                    )
            );
            return;
        }

        if (intakeDistance.getReading()) {
            intakeDistance.stop();

            Scheduler.getInstance().schedule(
                new Sequential(
                        new Wait(COLOR_DETECTION_DELAY),
                        new Instant(() -> isEmpty = false),
                        new Race(
                                new Command()
                                        .setExecute(this::updateInternalColors)
                                        .setDone(() -> hasThree),
                                new Sequential(
                                    new Wait(COLOR_DETECTION_PERIOD),
                                    new Instant(intakeDistance::start)
                                )
                        )
                )
            );
        }
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
        int balls = 0;
        for (ArtifactColor color : relativeCompartments)
            if (!color.equals(ArtifactColor.NONE))
                balls++;
        return balls + (!isEmpty ? 1 : 0);
    }

    public ArtifactColor[] getColors(){
        return relativeCompartments;
    }

    public void reset() {
        Arrays.fill(relativeCompartments, ArtifactColor.NONE);
        colorSensors.reset();
        intakeDistance.stop();
        hasThree = false;
        isEmpty = true;
        start();
    }

    public void start() {
        intakeDistance.start();
    }

    public boolean getIntakeDistanceState(){
        return intakeDistance.state();
    }

    public boolean isEmpty() {
        return isEmpty;
    }
}
