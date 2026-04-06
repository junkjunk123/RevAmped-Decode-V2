package org.firstinspires.ftc.teamcode.mechanisms.vision;

import androidx.annotation.NonNull;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.math.calc.Vector2D;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Tele;
import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RandomizationState;
import org.firstinspires.ftc.teamcode.utils.hardware.HwDevice;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;

public class DecodeLimelight implements HwDevice {
    public final Limelight3A limelight;
    private final String id;
    private LLResult latestResult;
    private Vector tagOffsets = new Vector();
    private long lastDetectionTime = 0;
    public static Vector2D TAG_OFFSETS = new Vector2D(1.8, 1);

    public static ColoredDecodePose APRILTAG_POSE = new ColoredDecodePose(14.5, 132, 0);
    public static double CENTER_OFFSET = 6.9;

    public enum Pipeline {
        NONE(-1),
        OBELISK(7),
        SHOOTING_ALIGNMENT(0),
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
        if (result != null && (result.isValid() || getCurrentPipeline() == Pipeline.SHOOTING_ALIGNMENT)) {
            latestResult = result;
            lastDetectionTime = result.getControlHubTimeStampNanos();
        }
    }

    public void update() {
        if (currentPipeline == Pipeline.NONE) return;
        computeLatestResult();
        if (latestResult == null) return;

        switch (currentPipeline) {
            case OBELISK -> {
                if (latestResult.getFiducialResults() == null || latestResult.getFiducialResults().isEmpty())
                    return;
                LLResultTypes.FiducialResult result = latestResult.getFiducialResults().get(0);
                int id = result.getFiducialId();
                for (RandomizationState state : RandomizationState.values()) {
                    if (id == state.getID()) {
                        Globals.randomizationState = state;
                        setCurrentPipeline(Pipeline.NONE);
                        return;
                    }
                }
            }
            case SHOOTING_ALIGNMENT -> {
                double[] output = latestResult.getPythonOutput();
                Globals.telemetry.addData("lloutput", Arrays.toString(output));
                if (Globals.allianceColor.getTagID() != output[0]) return;
                tagOffsets = new Vector2D(output[2], output[1]);
                setCurrentPipeline(Pipeline.NONE);
            }
        }
    }

    public Vector getOffsets() {
        return tagOffsets;
    }

    public long getLastDetectionTimeNanos() {
        return lastDetectionTime;
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
                .setDone(() -> Globals.randomizationState != null);
    }

    public ICommand computeOffsets() {
        return new Command()
                .setStart(() -> setCurrentPipeline(Pipeline.SHOOTING_ALIGNMENT))
                .setExecute(this::update)
                .setDone(() -> getCurrentPipeline() != Pipeline.SHOOTING_ALIGNMENT);
    }
}
