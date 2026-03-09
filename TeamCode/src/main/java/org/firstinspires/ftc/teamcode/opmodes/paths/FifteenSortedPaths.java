package org.firstinspires.ftc.teamcode.opmodes.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class FifteenSortedPaths implements PathSupplier {
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(31.5, 134, -Math.PI / 2);
    public static ColoredDecodePose FIRST_SHOOT_POSE = new ColoredDecodePose(56, 80, Math.toRadians(-60));
    public static ColoredDecodePose CONTROL_POINT_1 = new ColoredDecodePose(31.5, 123, Math.toRadians(-66));
    public static ColoredDecodePose SHOOT_POSE = new ColoredDecodePose(56, 78, Math.toRadians(204));
    public static ColoredDecodePose INTAKE_1 = new ColoredDecodePose(12, 59, Math.PI);
    public static ColoredDecodePose INTAKE_1_CONTROL = new ColoredDecodePose(45,59, Math.PI);
    public static ColoredDecodePose GATE_CONTROL = new ColoredDecodePose(56, 58);
    public static ColoredDecodePose GATE = new ColoredDecodePose(13, 58.5, Math.toRadians(150));
    public static ColoredDecodePose INTAKE_SECOND_PRELOAD_CONTROL = new ColoredDecodePose(52, 84, Math.PI);
    public static ColoredDecodePose INTAKE_SECOND_PRELOAD = new ColoredDecodePose(18, 84, Math.PI);
    public static ColoredDecodePose SECOND_PRELOAD_SHOOT = new ColoredDecodePose(60, 102, Math.toRadians(240));
    public static ColoredDecodePose THIRD_INTAKE_ACTUAL_START = new ColoredDecodePose(52,39);
    public static ColoredDecodePose THIRD_INTAKE_START = new ColoredDecodePose(52, 56, Math.toRadians(180));
    public static ColoredDecodePose THIRD_INTAKE_END = new ColoredDecodePose(12, 39, Math.toRadians(180));
    public static ColoredDecodePose PARK = new ColoredDecodePose(53, 112, Math.toRadians(220));

    @Override
    public Pose startPose() {
        return START_POSE.getPose();
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {
        FollowParameters shootPreloads = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_POSE, CONTROL_POINT_1, FIRST_SHOOT_POSE))
                .setConstantHeadingInterpolation(FIRST_SHOOT_POSE.getHeading())
                .build()
        );

        FollowParameters intakeFirstSet = new FollowParameters(Constants.FORWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_POSE, INTAKE_1_CONTROL, INTAKE_1))
                .setTangentHeadingInterpolation()
                .build()
        );

        FollowParameters shootFromGate = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE, SHOOT_POSE))
                .setConstantHeadingInterpolation(SHOOT_POSE.getHeading())
                .build()
        );

        FollowParameters shootFirstSet = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(INTAKE_1, SHOOT_POSE))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters intakeToGate = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_POSE, GATE_CONTROL, GATE))
                .setConstantHeadingInterpolation(GATE.getHeading())
                .build()
        );

        FollowParameters intakeSecondPresets = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_POSE, INTAKE_SECOND_PRELOAD_CONTROL, INTAKE_SECOND_PRELOAD))
                .setConstantHeadingInterpolation(INTAKE_SECOND_PRELOAD.getHeading())
                .build()
        );

        FollowParameters shootSecondSet = new FollowParameters(Constants.FORWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(INTAKE_SECOND_PRELOAD, SECOND_PRELOAD_SHOOT))
                .setConstantHeadingInterpolation(SECOND_PRELOAD_SHOOT.getHeading())
                .build()
        );

        FollowParameters intakeThirdSet = new FollowParameters(Constants.FORWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_PRELOAD_SHOOT, THIRD_INTAKE_START))
                .setLinearHeadingInterpolation(SECOND_PRELOAD_SHOOT.getHeading(), THIRD_INTAKE_END.getHeading(), 0.8)
                .addPath(ColoredDecodePose.makeBezier(THIRD_INTAKE_ACTUAL_START, THIRD_INTAKE_END))
                .setConstantHeadingInterpolation(THIRD_INTAKE_END.getHeading())
                .build()
        );

        FollowParameters park = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(new BezierLine(THIRD_INTAKE_END, PARK))
                .setConstantHeadingInterpolation(PARK.getHeading())
                .build()
        );

        return List.of(shootPreloads, intakeFirstSet, shootFirstSet, intakeToGate, shootFromGate, intakeSecondPresets, shootSecondSet,
                intakeThirdSet, park);
    }
}
