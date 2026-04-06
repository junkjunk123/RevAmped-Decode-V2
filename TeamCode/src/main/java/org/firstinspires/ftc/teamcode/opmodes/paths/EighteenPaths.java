package org.firstinspires.ftc.teamcode.opmodes.paths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;

import java.util.List;

@Configurable
public class EighteenPaths implements PathSupplier {
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(45, 8.75, Math.toRadians(180));

    public static ColoredDecodePose FIRST_INTAKE = new ColoredDecodePose(10, 9, Math.toRadians(180));
    public static ColoredDecodePose FIRST_SHOOT = new ColoredDecodePose(55, 18, Math.toRadians(140));
    public static ColoredDecodePose SECOND_INTAKE_CONTROL = new ColoredDecodePose(52, 58);
    public static ColoredDecodePose SECOND_INTAKE = new ColoredDecodePose(14, 58);

    public static ColoredDecodePose SECOND_SHOOT_CONTROL = new ColoredDecodePose(40, 60);
    public static ColoredDecodePose SECOND_SHOOT = new ColoredDecodePose(57, 77);

    public static ColoredDecodePose GATE_SHOOT_CONTROL = new ColoredDecodePose(33, 62);
    public static ColoredDecodePose GATE_SHOOT = new ColoredDecodePose(50, 88);

    public static ColoredDecodePose GATE_INTAKE = new ColoredDecodePose(9.5, 59, Math.toRadians(150));

    public static ColoredDecodePose THIRD_INTAKE_CONTROL = new ColoredDecodePose(48, 84);
    public static ColoredDecodePose THIRD_INTAKE = new ColoredDecodePose(19, 84, Math.toRadians(180));

    public static ColoredDecodePose THIRD_SHOOT = new ColoredDecodePose(50, 88, Math.toRadians(225));

    public static ColoredDecodePose FOURTH_INTAKE_CONTROL = new ColoredDecodePose(18, 67);
    public static ColoredDecodePose FOURTH_INTAKE = new ColoredDecodePose(20, 38);

    public static ColoredDecodePose PARK = new ColoredDecodePose(41, 73);

    @Override
    public Pose startPose() {
        return START_POSE.getPose();
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {
        FollowParameters path1 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_POSE, FIRST_INTAKE))
                .setConstantHeadingInterpolation(FIRST_INTAKE.getHeading())
                .build()
        );

        FollowParameters path2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_INTAKE, FIRST_SHOOT))
                .setConstantHeadingInterpolation(FIRST_SHOOT.getHeading())
                .build()
        );

        FollowParameters path3 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_SHOOT, SECOND_INTAKE_CONTROL, SECOND_INTAKE))
                .setTangentHeadingInterpolation()
                .build()
        );

        FollowParameters path4 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_INTAKE, SECOND_SHOOT_CONTROL, SECOND_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters path5 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_SHOOT, GATE_INTAKE))
                .setConstantHeadingInterpolation(GATE_INTAKE.getHeading())
                .build()
        );

        FollowParameters path6 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_INTAKE, GATE_SHOOT_CONTROL, GATE_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters path7 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_SHOOT, THIRD_INTAKE_CONTROL, THIRD_INTAKE))
                .setConstantHeadingInterpolation(THIRD_INTAKE.getHeading())
                .build()
        );

        FollowParameters path8 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(THIRD_INTAKE, THIRD_SHOOT))
                .setLinearHeadingInterpolation(THIRD_INTAKE.getHeading(), THIRD_SHOOT.getHeading())
                .build()
        );

        FollowParameters path9 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(THIRD_SHOOT, FOURTH_INTAKE_CONTROL, FOURTH_INTAKE))
                .setTangentHeadingInterpolation()
                .build()
        );

        FollowParameters path10 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FOURTH_INTAKE, THIRD_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters path11 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(THIRD_SHOOT, PARK))
                .setTangentHeadingInterpolation()
                .build()
        );

        return List.of(
                path1,
                path2,
                path3,
                path4,
                path5,
                path6,
                path7,
                path8,
                path9,
                path10,
                path11
        );
    }
}