package org.firstinspires.ftc.teamcode.opmodes.paths;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;

import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

@Configurable
public class UnsortedCloseAutoPaths implements PathSupplier {
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(31.5, 134, -Math.PI / 2);
    public static ColoredDecodePose FIRST_SHOOT_POSE = new ColoredDecodePose(56, 80, Math.toRadians(-60));
    public static ColoredDecodePose CONTROL_POINT_1 = new ColoredDecodePose(48.5, 56, Math.toRadians(-66));
    public static ColoredDecodePose SHOOT_POSE = new ColoredDecodePose(56, 78, Math.toRadians(204));
    public static ColoredDecodePose INTAKE_1 = new ColoredDecodePose(15, 61, Math.PI);
    public static ColoredDecodePose INTAKE_1_CONTROL = new ColoredDecodePose(45,59, Math.PI);
    public static ColoredDecodePose GATE_START = new ColoredDecodePose(20, 61, Math.toRadians(200));
    public static ColoredDecodePose GATE_HOLD = new ColoredDecodePose(21, 63.5, Math.toRadians(190));
    public static ColoredDecodePose GATE = new ColoredDecodePose(13, 58, Math.toRadians(150));
    public static ColoredDecodePose GATE_1 = new ColoredDecodePose(13, 58, Math.toRadians(150));
    public static ColoredDecodePose GATE_2 = new ColoredDecodePose(13, 58.5, Math.toRadians(150));
    public static ColoredDecodePose GATE_3 = new ColoredDecodePose(13.5, 58.5, Math.toRadians(150));
    public static ColoredDecodePose INTAKE_FINAL_PRELOAD_CONTROL = new ColoredDecodePose(52, 84, Math.PI);
    public static ColoredDecodePose INTAKE_FINAL_PRELOAD = new ColoredDecodePose(23.5, 84, Math.PI);
    public static ColoredDecodePose PARK_CONTROL = new ColoredDecodePose(30.5, 84);
    public static ColoredDecodePose PARK = new ColoredDecodePose(53, 112, Math.toRadians(220));

    @Override
    public Pose startPose() {
        return START_POSE.getPose();
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {
        FollowParameters shootPreloads = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_POSE, CONTROL_POINT_1, FIRST_SHOOT_POSE))
                .setConstantHeadingInterpolation(FIRST_SHOOT_POSE.getHeading())
                .build()
        );

        FollowParameters intakeFirstSet = new FollowParameters(Constants.CONSERVATIVE_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_SHOOT_POSE, INTAKE_1_CONTROL, INTAKE_1))
                .setTangentHeadingInterpolation()
                .build()
        );

        PathChain shootingFromGate = follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE, SHOOT_POSE))
                .setConstantHeadingInterpolation(SHOOT_POSE.getHeading())
                .build();

        FollowParameters shootFirstSet = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(INTAKE_1, SHOOT_POSE))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        Function<Integer, FollowParameters> intakeToGate = getIntakeToGate(follower);

        FollowParameters holdGate = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(getGatePose(0), GATE_HOLD))
                .setLinearHeadingInterpolation(getGatePose(0).getHeading(), GATE_HOLD.getHeading())
                .build()
        );

        Supplier<FollowParameters> shootFromGate = () -> new FollowParameters(Constants.DEFAULT_PROPORTIONAL, shootingFromGate);

        FollowParameters intakeFinalPresets = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_POSE, INTAKE_FINAL_PRELOAD_CONTROL, INTAKE_FINAL_PRELOAD))
                .setConstantHeadingInterpolation(INTAKE_FINAL_PRELOAD.getHeading())
                .build()
        );

        FollowParameters shootFinalPresets = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(INTAKE_FINAL_PRELOAD, PARK_CONTROL, PARK))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        return List.of(shootPreloads, intakeFirstSet, shootFirstSet, intakeToGate.apply(0), holdGate, shootFromGate.get(),
                intakeToGate.apply(1), shootFromGate.get(), intakeToGate.apply(2), shootFromGate.get(), intakeToGate.apply(3),
                shootFromGate.get(), intakeFinalPresets, shootFinalPresets);
    }

    @NonNull
    private static Function<Integer, FollowParameters> getIntakeToGate(Follower follower) {
        return i -> new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_POSE, GATE_START))
                .setTangentHeadingInterpolation()
                .addPath(ColoredDecodePose.makeBezier(GATE_START, getGatePose(i)))
                .setLinearHeadingInterpolation(GATE_START.getHeading(), getGatePose(i).getHeading())
                .build()
        );
    }

    private static ColoredDecodePose getGatePose(int iteration) {
        return switch (iteration) {
            case 0 -> GATE;
            case 1 -> GATE_1;
            case 2 -> GATE_2;
            case 3 -> GATE_3;
            default -> throw new IllegalStateException("Unexpected value: " + iteration);
        };
    }
}
