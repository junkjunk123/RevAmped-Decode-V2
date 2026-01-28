package org.firstinspires.ftc.teamcode.mechanisms.intake;

import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

import java.util.Arrays;
import java.util.stream.IntStream;

public class TableCompartmentManager {
    public final ArtifactColor[] compartmentColors = new ArtifactColor[] {
            ArtifactColor.NONE,
            ArtifactColor.NONE,
            ArtifactColor.NONE
    };

    public final IntakeThread intakeThread;

    public TableCompartmentManager(ColorManager colorManager) {
        RobotStateHandler.CycleState.INTAKE.init(colorManager, compartmentColors);
        intakeThread = RobotStateHandler.CycleState.INTAKE.intakeThread;
    }

    public boolean isEmpty() {
        return Arrays.stream(compartmentColors).allMatch(t -> t == ArtifactColor.NONE);
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
        if (isEmpty() || allGreen() || allPurple() || Globals.randomizationState == null) {
            return curIndex;
        }

        int targetGreenIndex = Globals.randomizationState.getGreenIndex();
        int curGreenIndex = getGreenIndices()[0];

        return (curGreenIndex - targetGreenIndex + 3) % 3;
    }

    public void populate(ArtifactColor[] colors) {
        compartmentColors[0] = colors[0];
        compartmentColors[1] = colors[1];
        compartmentColors[2] = colors[2];
    }

    public void removeAll() {
        compartmentColors[0] = ArtifactColor.NONE;
        compartmentColors[1] = ArtifactColor.NONE;
        compartmentColors[2] = ArtifactColor.NONE;
    }
}
