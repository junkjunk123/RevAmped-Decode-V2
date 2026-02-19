package org.firstinspires.ftc.teamcode.mechanisms.vision;

import androidx.annotation.NonNull;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RandomizationState;
import org.firstinspires.ftc.teamcode.utils.hardware.HwDevice;
import org.firstinspires.ftc.teamcode.utils.logging.DecodeLogger;

import java.util.List;

public class DecodeLimelight implements HwDevice {
    private static final long STALE_THRESHOLD_MS = 750;
    public final Limelight3A limelight;
    private final String id;
    private LLResult latestResult;
    private Pose detectionResult = new Pose();
    private long lastDetectionTimeNanos;
    private long lastStaleWarningNanos;
    private boolean lastShootingTargetVisible;

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
        Pipeline next = pipeline == null ? Pipeline.NONE : pipeline;
        if (currentPipeline == next) return;

        latestResult = null;
        detectionResult = new Pose();
        lastDetectionTimeNanos = 0;
        lastStaleWarningNanos = 0;
        lastShootingTargetVisible = false;

        if (next != Pipeline.NONE) {
            limelight.pipelineSwitch(next.getPipeline());
            limelight.start();
        } else limelight.pause();
        currentPipeline = next;
        DecodeLogger.get().info("vision", "LIMELIGHT_PIPELINE_SET", "pipeline", next.name());
    }

    private void computeLatestResult() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            latestResult = result;
            lastDetectionTimeNanos = System.nanoTime();
        } else {
            latestResult = null;
        }
    }

    public void update() {
        if (currentPipeline == Pipeline.NONE) return;
        computeLatestResult();
        warnIfStale();
        if (latestResult == null) {
            if (currentPipeline == Pipeline.SHOOTING_ALIGNMENT && lastShootingTargetVisible) {
                detectionResult = new Pose();
                DecodeLogger.get().info("vision", "LIMELIGHT_DETECTION",
                        "detected", false,
                        "pipeline", currentPipeline.name(),
                        "tagId", Globals.allianceColor.getTagID());
                lastShootingTargetVisible = false;
            }
            return;
        }

        switch (currentPipeline) {
            case OBELISK -> {
                if (latestResult.getFiducialResults() == null || latestResult.getFiducialResults().isEmpty())
                    return;
                LLResultTypes.FiducialResult result = latestResult.getFiducialResults().get(0);
                int id = result.getFiducialId();
                for (RandomizationState state : RandomizationState.values()) {
                    if (id == state.getID()) {
                        Globals.randomizationState = state;
                        DecodeLogger.get().info("vision", "LIMELIGHT_DETECTION",
                                "detected", true,
                                "pipeline", currentPipeline.name(),
                                "state", state.name(),
                                "fiducialId", id);
                        setCurrentPipeline(Pipeline.NONE);
                        return;
                    }
                }
            }

            case SHOOTING_ALIGNMENT -> {
                double tagID = Globals.allianceColor.getTagID();
                List<LLResultTypes.FiducialResult> r = latestResult.getFiducialResults();
                LLResultTypes.FiducialResult target = null;
                if (r != null && !r.isEmpty()) {
                    for (LLResultTypes.FiducialResult i : r) {
                        if (i != null && i.getFiducialId() == tagID) {
                            target = i;
                            break;
                        }
                    }
                }

                if (target != null) {
                    double x = target.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH).x; // right/left from tag
                    double z = target.getCameraPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH).z; // forward/back from tag
                    double h = target.getCameraPoseTargetSpace().getOrientation().getYaw(AngleUnit.RADIANS); // heading from tag

                    detectionResult = new Pose(x, z, h, FTCCoordinates.INSTANCE);
                    if (!lastShootingTargetVisible) {
                        DecodeLogger.get().info("vision", "LIMELIGHT_DETECTION",
                                "detected", true,
                                "pipeline", currentPipeline.name(),
                                "tagId", tagID);
                    }
                    lastShootingTargetVisible = true;
                    return;
                }

                detectionResult = new Pose();
                if (lastShootingTargetVisible) {
                    DecodeLogger.get().info("vision", "LIMELIGHT_DETECTION",
                            "detected", false,
                            "pipeline", currentPipeline.name(),
                            "tagId", tagID);
                }
                lastShootingTargetVisible = false;
            }
        }
    };

    public Pose getDetectionResult() {
        return detectionResult;
    }

    public long getLastDetectionTimeNanos() {
        return lastDetectionTimeNanos;
    }

    public Pipeline getCurrentPipeline() {
        return currentPipeline;
    }

    public void close() {
        setCurrentPipeline(Pipeline.NONE);
        limelight.close();
    }

    public Command detectMotif() {
        return new Command()
                .setStart(() -> setCurrentPipeline(Pipeline.OBELISK))
                .setExecute(this::update)
                .setDone(() -> getCurrentPipeline() != Pipeline.OBELISK);
    }

    private void warnIfStale() {
        if (lastDetectionTimeNanos == 0) return;
        long nowNanos = System.nanoTime();
        long staleMs = (nowNanos - lastDetectionTimeNanos) / 1_000_000L;
        if (staleMs > STALE_THRESHOLD_MS && nowNanos - lastStaleWarningNanos > STALE_THRESHOLD_MS * 1_000_000L) {
            DecodeLogger.get().warn("vision", "SENSOR_STALE",
                    "sensor", "limelight",
                    "staleMs", staleMs,
                    "pipeline", currentPipeline.name());
            lastStaleWarningNanos = nowNanos;
        }
    }
}
