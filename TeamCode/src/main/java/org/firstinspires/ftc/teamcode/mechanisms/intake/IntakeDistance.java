package org.firstinspires.ftc.teamcode.mechanisms.intake;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utils.data.BooleanPeriod;
import org.firstinspires.ftc.teamcode.utils.control.FirstOrderLowPass;
import org.firstinspires.ftc.teamcode.utils.hardware.HwDistance;

//Eric Debug Thanks
public class IntakeDistance extends HwDistance {
    public static double PERIOD_HOLD; //ms
    public static double PERIOD_PASS_THROUGH; //ms
    public static double PERIOD_PASS_THROUGH_2;
    public static double MAX_DIST;
    private final FirstOrderLowPass lowPass;
    private final BooleanPeriod compartmentThree;
    private final BooleanPeriod artifactPassThrough;
    private final BooleanPeriod artifactPassThroughPhaseTwo;

    public enum DetectionState {
        NONE,
        PASS_THROUGH_1,
        PASS_THROUGH_2,
        COMPARTMENT_THREE
    }

    private DetectionState detectionState = DetectionState.NONE;

    public IntakeDistance(HardwareMap hardwareMap) {
        super(hardwareMap, "intakeDistance");
        lowPass = new FirstOrderLowPass(0.6);
        compartmentThree = new BooleanPeriod(() -> getDistanceFiltered() <= MAX_DIST, PERIOD_HOLD);
        artifactPassThrough = new BooleanPeriod(() -> getDistanceFiltered() <= MAX_DIST, PERIOD_PASS_THROUGH);
        artifactPassThroughPhaseTwo = new BooleanPeriod(() -> getDistanceFiltered() >= MAX_DIST, PERIOD_PASS_THROUGH_2);
    }

    public boolean hasArtifact() {
        return compartmentThree.getAsBoolean();
    }

    public boolean detectedArtifact() {
        return compartmentThree.isLastValue();
    }

    private boolean artifactPassThrough() {
        return artifactPassThrough.getAsBoolean();
    }

    public boolean artifactPassedThrough() {
        return detectionState.equals(DetectionState.PASS_THROUGH_2) && artifactPassThroughPhaseTwo.getAsBoolean();
    }

    public void setDetectionState(DetectionState detectionState) {
        this.detectionState = detectionState;
        switch (detectionState) {
            case PASS_THROUGH_1 -> artifactPassThrough.start();
            case PASS_THROUGH_2 -> artifactPassThroughPhaseTwo.start();
            case COMPARTMENT_THREE -> compartmentThree.start();
        }
    }

    public DetectionState getDetectionState() {
        return detectionState;
    }

    @Override
    public void update() {
        switch (detectionState) {
            case PASS_THROUGH_1 -> {
                artifactPassThrough.update();
                if (artifactPassThrough.getAsBoolean()) setDetectionState(DetectionState.PASS_THROUGH_2);
            }
            case PASS_THROUGH_2 -> artifactPassThroughPhaseTwo.update();
            case COMPARTMENT_THREE -> compartmentThree.update();
        }
    }

    public double getDistanceFiltered() {
        return lowPass.update(getDistance());
    }
}
