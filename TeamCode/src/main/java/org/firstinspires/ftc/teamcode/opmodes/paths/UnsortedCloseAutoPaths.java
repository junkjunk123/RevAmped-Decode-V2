package org.firstinspires.ftc.teamcode.opmodes.paths;
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
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(14, 114, 0);
    public static ColoredDecodePose FIRST_SHOOT_POSE = new ColoredDecodePose(59, 80, Math.PI);
    public static ColoredDecodePose CONTROL_POINT_1 = new ColoredDecodePose(50, 98, Math.PI);
    public static ColoredDecodePose SHOOT_POSE = new ColoredDecodePose(57, 78, Math.PI);
    public static ColoredDecodePose INTAKE_1 = new ColoredDecodePose(12, 59, Math.PI);
    public static ColoredDecodePose INTAKE_1_CONTROL = new ColoredDecodePose(50, 59, Math.PI);
    public static ColoredDecodePose SHOOT_POSE_CONTROL_1 = new ColoredDecodePose(32, 58, Math.PI);
    public static ColoredDecodePose GATE = new ColoredDecodePose(12, 61, Math.toRadians(155));
    public static ColoredDecodePose INTAKE_FINAL_PRELOAD_CONTROL = new ColoredDecodePose(55, 85, Math.PI);
    public static ColoredDecodePose INTAKE_FINAL_PRELOAD = new ColoredDecodePose(18, 84, Math.PI);
    public static ColoredDecodePose PARK = new ColoredDecodePose(56, 111, Math.toRadians(220));

    @Override
    public Pose startPose() {
        return START_POSE.getPose();
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {
        FollowParameters shootPreloads = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_POSE, CONTROL_POINT_1, FIRST_SHOOT_POSE))
                .setTangentHeadingInterpolation()
                .build()
        );

        FollowParameters intakeFirstSet = new FollowParameters(Constants.FORWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_POSE, INTAKE_1_CONTROL, INTAKE_1))
                .setTangentHeadingInterpolation()
                .build()
        );

        PathChain shootingFromGate = follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE, SHOOT_POSE))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        FollowParameters shootFirstSet = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(INTAKE_1, SHOOT_POSE_CONTROL_1, SHOOT_POSE))
                .setConstantHeadingInterpolation(shootingFromGate.getFinalHeadingGoal())
                .build()
        );

        Function<Integer, ColoredDecodePose> getGatePose = i -> GATE;

        Function<Integer, FollowParameters> intakeToGate = i -> new FollowParameters(Constants.FORWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_POSE, INTAKE_1_CONTROL, getGatePose.apply(i)))
                .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), GATE.getHeading(), 0.8)
                .build()
        );

        Supplier<FollowParameters> shootFromGate = () -> new FollowParameters(Constants.BACKWARD_PROPORTIONAL, shootingFromGate);

        FollowParameters intakeFinalPresets = new FollowParameters(Constants.FORWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_POSE, INTAKE_FINAL_PRELOAD_CONTROL, INTAKE_FINAL_PRELOAD))
                .setConstantHeadingInterpolation(INTAKE_FINAL_PRELOAD.getHeading())
                .build()
        );

        FollowParameters shootFinalPresets = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(INTAKE_FINAL_PRELOAD, PARK))
                .setLinearHeadingInterpolation(INTAKE_FINAL_PRELOAD.getHeading(), PARK.getHeading(), 0.8)
                .build()
        );

        return List.of(shootPreloads, intakeFirstSet, shootFirstSet, intakeToGate.apply(0), shootFromGate.get(),
                intakeToGate.apply(1), shootFromGate.get(), intakeToGate.apply(2), shootFromGate.get(),
                intakeFinalPresets, shootFinalPresets);
    }
}
