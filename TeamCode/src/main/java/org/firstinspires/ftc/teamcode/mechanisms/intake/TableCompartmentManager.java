package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;

import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.SpindexerColorSensors;
import org.firstinspires.ftc.teamcode.utils.ArtifactColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;
import java.util.stream.IntStream;

public class TableCompartmentManager {
    public final ArtifactColor[] compartmentColors = new ArtifactColor[] {
            ArtifactColor.NONE,
            ArtifactColor.NONE,
            ArtifactColor.NONE
    };

    public final IntakeThread intakeThread;
    private final IntakeArtifactDetector intakeDistance;
    private final IntakeArtifactDetector frontCompartmentSensor;
    private final Supplier<Table.RelativeState> tableState;

    public TableCompartmentManager(SpindexerColorSensors colorManager, IntakeArtifactDetector intakeDetector,
                                   IntakeArtifactDetector frontSensor, Supplier<Table.RelativeState> tableState) {
        this.tableState = tableState;
        RobotStateHandler.CycleState.INTAKE.init(colorManager, compartmentColors, intakeDetector, frontSensor);
        intakeThread = RobotStateHandler.CycleState.INTAKE.intakeThread;
        this.intakeDistance = intakeDetector;
        this.frontCompartmentSensor = frontSensor;
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

    public int sort() {
        int curIndex = tableState.get().ordinal();
        if (isEmpty() || allGreen() || allPurple() || Globals.randomizationState == null) {
            return curIndex;
        }

        int targetGreenIndex = Globals.randomizationState.getGreenIndex();
        int curGreenIndex = getGreenIndices()[0];

        return (curGreenIndex - targetGreenIndex + 3) % 3;
    }

    public void populate(ArtifactColor... colors) {
        compartmentColors[0] = colors[0];
        compartmentColors[1] = colors[1];
        compartmentColors[2] = colors[2];
    }

    public void populate() {
        intakeThread.updateColors(tableState.get().ordinal());
    }

    public ICommand populateAuto() {
        AtomicBoolean hasLeft = new AtomicBoolean(false);
        AtomicBoolean hasRight = new AtomicBoolean(false);

        return new Command()
                .setStart(() -> {
                    compartmentColors[2] = intakeThread.colorSensors.leftColorSensor.getColor();
                    compartmentColors[0] = intakeThread.colorSensors.rightColorSensor.getColor();
                    hasLeft.set(compartmentColors[2].equals(ArtifactColor.NONE));
                    hasRight.set(compartmentColors[0].equals(ArtifactColor.NONE));
                })
                .setExecute(() -> {
                    if (!hasLeft.get()) compartmentColors[2] = intakeThread.colorSensors.leftColorSensor.getColor();
                    if (!hasRight.get()) compartmentColors[0] = intakeThread.colorSensors.rightColorSensor.getColor();
                })
                .setDone(() -> hasLeft.get() && hasRight.get());
    }

    public void removeAll() {
        compartmentColors[0] = ArtifactColor.NONE;
        compartmentColors[1] = ArtifactColor.NONE;
        compartmentColors[2] = ArtifactColor.NONE;
        intakeThread.reset();
    }
}
