package org.firstinspires.ftc.teamcode.revamped.mechanisms;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.revamped.utils.Globals;
import org.firstinspires.ftc.teamcode.revamped.utils.RandomizationState;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.HwDevice;

import java.util.List;

import dev.frozenmilk.dairy.mercurial.continuations.Continuation;

public class DecodeLimelight implements HwDevice {
    public final Limelight3A limelight;
    private final String id;
    private LLResult latestResult;
    private Pose detectionResult;
    public static boolean usingLimelight = false;
    private long lastDetectionTime = 0;

    public enum Pipeline {
        NONE(-1),
        OBELISK(7),
        SHOOTING_ALIGNMENT(1),
        ELEMENT_DETECTION(2);

        final int pipeline;

        Pipeline(int pipeline) {
            this.pipeline = pipeline;
        }

        public int getPipeline() {
            return pipeline;
        }
    }
    private Pipeline currentPipeline = Pipeline.NONE;

    public DecodeLimelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(10);
        this.id = "limelight";
    }

    @NonNull
    @Override
    public String toString() {
        return id;
    }

    public void setCurrentPipeline(Pipeline pipeline) {
        if (pipeline != Pipeline.NONE) {
            limelight.pipelineSwitch(pipeline.getPipeline());
            limelight.start();
        }
        else limelight.pause();
        currentPipeline = pipeline;
    }

    private void computeLatestResult() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            latestResult = result;
            lastDetectionTime = result.getControlHubTimeStampNanos();
        }
    }

    public Continuation update = new Continuation() {
            @NonNull
            @Override
            public StackTraceElement[] getStackTrace() {
                return new StackTraceElement[] {new StackTraceElement(
                        "DecodeLimelight", "update", "DecodeLimelight.java", 89)};
            }

            @NonNull
            @Override
            public Continuation apply() {
                if (currentPipeline == Pipeline.NONE) return update;
                computeLatestResult();
                if (latestResult == null) return update;

                switch (currentPipeline) {
                    case OBELISK -> {
                        if (latestResult.getFiducialResults() == null || latestResult.getFiducialResults().isEmpty())
                            return update;
                        LLResultTypes.FiducialResult result = latestResult.getFiducialResults().get(0);
                        int id = result.getFiducialId();
                        for (RandomizationState state : RandomizationState.values()) {
                            if (id == state.getID()) {
                                Globals.randomizationState = state;
                                setCurrentPipeline(Pipeline.NONE);
                                return update;
                            }
                        }
                    }

                    case SHOOTING_ALIGNMENT -> {
                        double tagID = Globals.allianceColor.getTagID();
                        List<LLResultTypes.FiducialResult> r = latestResult.getFiducialResults();
                        LLResultTypes.FiducialResult target = null;
                        for (LLResultTypes.FiducialResult i : r) {
                            if (i != null && i.getFiducialId() == tagID) {
                                target = i;
                                break;
                            }
                        }

                        if (target != null) {
                            double x = target.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH).x; // right/left from tag
                            double z = target.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH).z; // forward/back from tag
                            double h = target.getCameraPoseTargetSpace().getOrientation().getYaw(AngleUnit.RADIANS); // heading from tag

                            detectionResult = new Pose(x, z, h, FTCCoordinates.INSTANCE);
                            return update;
                        }

                        detectionResult = new Pose();
                    }
                }

                return update;
            }
    };

    public Pose getDetectionResult() {
        return detectionResult;
    }

    public long getLastDetectionTimeNanos() {
        return lastDetectionTime;
    }

    public Pipeline getCurrentPipeline() {
        return currentPipeline;
    }

    public void close() {
        limelight.close();
    }
}
