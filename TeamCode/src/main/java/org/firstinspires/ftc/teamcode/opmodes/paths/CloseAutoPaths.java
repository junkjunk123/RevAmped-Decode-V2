package org.firstinspires.ftc.teamcode.opmodes.paths;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;

import java.util.List;

@Config
public class CloseAutoPaths implements PathSupplier {
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(34.5, 136, Math.toRadians(-90));
    public static ColoredDecodePose SHOOT_PRELOADS = new ColoredDecodePose(60, 102, Math.toRadians(249));
    public static ColoredDecodePose FIRST_INTAKE_START = new ColoredDecodePose(56, 83, Math.toRadians(180));
    public static ColoredDecodePose FIRST_INTAKE_END = new ColoredDecodePose(28, 83, Math.toRadians(180));
    public static ColoredDecodePose FIRST_SHOOT = new ColoredDecodePose(60, 102, Math.toRadians(240));
    public static ColoredDecodePose SECOND_INTAKE_START = new ColoredDecodePose(53, 58, Math.toRadians(180));
    public static ColoredDecodePose SECOND_INTAKE_END = new ColoredDecodePose(25, 58, Math.toRadians(180));
    public static ColoredDecodePose SECOND_SHOOT = new ColoredDecodePose(60, 102, Math.toRadians(240));
    public static ColoredDecodePose THIRD_INTAKE_START = new ColoredDecodePose(52, 36, Math.toRadians(180));
    public static ColoredDecodePose THIRD_INTAKE_END = new ColoredDecodePose(24, 36, Math.toRadians(180));
    public static ColoredDecodePose THIRD_SHOOT = new ColoredDecodePose(66, 108, Math.toRadians(240));
    public static ColoredDecodePose PARK = new ColoredDecodePose(52, 120, Math.toRadians(240));

    @Override
    public Pose startPose() {
        return START_POSE.getPose();
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {
        FollowParameters initialShoot = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_POSE, SHOOT_PRELOADS))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_PRELOADS.getHeading())
                .build()
        );

        FollowParameters intakeFirstSet = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_PRELOADS, FIRST_INTAKE_START))
                .setLinearHeadingInterpolation(SHOOT_PRELOADS.getHeading(), FIRST_INTAKE_END.getHeading(), 0.8)
                .addPath(ColoredDecodePose.makeBezier(FIRST_INTAKE_START, FIRST_INTAKE_END))
                .setConstantHeadingInterpolation(FIRST_INTAKE_END.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(0.5))
                .addParametricCallback(0.95, () -> follower.setMaxPower(1.0))
                .build()
        );

        FollowParameters shootFirstSet = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_INTAKE_END, FIRST_SHOOT))
                .setConstantHeadingInterpolation(FIRST_SHOOT.getHeading())
                .build()
        );

        FollowParameters intakeSecondSet = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_SHOOT, SECOND_INTAKE_START))
                .setLinearHeadingInterpolation(FIRST_SHOOT.getHeading(), SECOND_INTAKE_END.getHeading(), 0.8)
                .addPath(ColoredDecodePose.makeBezier(SECOND_INTAKE_START, SECOND_INTAKE_END))
                .setConstantHeadingInterpolation(SECOND_INTAKE_END.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(0.5))
                .addParametricCallback(0.95, () -> follower.setMaxPower(1.0))
                .build()
        );

        FollowParameters shootSecondSet = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_INTAKE_END, SECOND_SHOOT))
                .setConstantHeadingInterpolation(SECOND_SHOOT.getHeading())
                .build()
        );

        FollowParameters intakeThirdSet = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_SHOOT, THIRD_INTAKE_START))
                .setLinearHeadingInterpolation(SECOND_SHOOT.getHeading(), THIRD_INTAKE_END.getHeading(), 0.8)
                .addPath(ColoredDecodePose.makeBezier(THIRD_INTAKE_START, THIRD_INTAKE_END))
                .setConstantHeadingInterpolation(THIRD_INTAKE_END.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(0.5))
                .addParametricCallback(0.95, () -> follower.setMaxPower(1.0))
                .build()
        );

        FollowParameters shootThirdSet = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(THIRD_INTAKE_END, THIRD_SHOOT))
                .setConstantHeadingInterpolation(THIRD_SHOOT.getHeading())
                .build()
        );

        FollowParameters park = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(THIRD_SHOOT, PARK))
                .setConstantHeadingInterpolation(THIRD_SHOOT.getHeading())
                .build()
        );
        return List.of(initialShoot, intakeFirstSet, shootFirstSet, intakeSecondSet, shootSecondSet, intakeThirdSet, shootThirdSet, park);
    }
}
