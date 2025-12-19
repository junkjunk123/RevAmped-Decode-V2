package org.firstinspires.ftc.teamcode.revamped.mechanisms.intake;

import org.firstinspires.ftc.teamcode.revamped.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.revamped.utils.Globals;

import java.util.Arrays;
import java.util.stream.IntStream;

public class TableCompartmentManager {
    public final ArtifactColor[] compartmentColors = new ArtifactColor[] {
            ArtifactColor.NONE,
            ArtifactColor.NONE,
            ArtifactColor.NONE
    };

    public final IntakeThread intakeThread;

    public TableCompartmentManager(IntakeThread intakeThread) {
        this.intakeThread = intakeThread;
    }

    public boolean isEmpty() {
        return Arrays.stream(compartmentColors).anyMatch(t -> t != ArtifactColor.NONE);
    }

    public boolean isFull() {
        return Arrays.stream(compartmentColors).allMatch(t -> t != ArtifactColor.NONE);
    }

    public boolean allGreen() {
        return Arrays.stream(compartmentColors).allMatch(t -> t.equals(ArtifactColor.GREEN));
    }

    public boolean allPurple() {
        return Arrays.stream(compartmentColors).allMatch(t -> t.equals(ArtifactColor.PURPLE));
    }

    public int[] getGreenIndices() {
        return IntStream.range(0, 3)
                .filter(i -> compartmentColors[i].equals(ArtifactColor.GREEN))
                .toArray();
    }

    public int sort(int curIndex) {
        if (isEmpty() || allGreen() || allPurple() || Globals.randomizationState == null)
            return curIndex;

        int targetGreenIndex = Globals.randomizationState.getGreenIndex();
        int curGreenIndex = getGreenIndices()[0];

        return curGreenIndex - targetGreenIndex;
    }

    public void populate(ArtifactColor[] colors) {
        System.arraycopy(colors, 0, compartmentColors, 0, compartmentColors.length);
    }
}
