package org.firstinspires.ftc.teamcode.opmodes.paths;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;

import java.util.List;

@Config
public class CloseAutoPaths implements PathSupplier {
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(34.5, 136, Math.toRadians(-90));
    public static ColoredDecodePose PARTNER_PUSH = new ColoredDecodePose(24, 125, Math.toRadians(232));
    public static ColoredDecodePose SHOOT_PRELOADS = new ColoredDecodePose(60, 102, Math.toRadians(249));
    public static ColoredDecodePose FIRST_INTAKE_START = new ColoredDecodePose(56, 89, Math.toRadians(180));
    public static ColoredDecodePose FIRST_INTAKE_END = new ColoredDecodePose(22, 83, Math.toRadians(180));
    public static ColoredDecodePose FIRST_SHOOT = new ColoredDecodePose(60, 102, Math.toRadians(240));
    public static ColoredDecodePose SECOND_INTAKE_ACTUAL_START = new ColoredDecodePose(53, 60);
    public static ColoredDecodePose SECOND_INTAKE_START = new ColoredDecodePose(53, 77, Math.toRadians(180));
    public static ColoredDecodePose SECOND_INTAKE_END = new ColoredDecodePose(13, 60, Math.toRadians(180));
    public static ColoredDecodePose SECOND_SHOOT = new ColoredDecodePose(60, 102, Math.toRadians(240));
    public static ColoredDecodePose THIRD_INTAKE_ACTUAL_START = new ColoredDecodePose(52,39);
    public static ColoredDecodePose THIRD_INTAKE_START = new ColoredDecodePose(52, 56, Math.toRadians(180));
    public static ColoredDecodePose THIRD_INTAKE_END = new ColoredDecodePose(12, 39, Math.toRadians(180));
    public static ColoredDecodePose THIRD_SHOOT = new ColoredDecodePose(66, 108, Math.toRadians(240));
    public static ColoredDecodePose PARK = new ColoredDecodePose(52, 120, Math.toRadians(240));

    @Override
    public Pose startPose() {
        return START_POSE.getPose();
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {
        FollowParameters initialShoot = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_POSE, FIRST_SHOOT))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), FIRST_SHOOT.getHeading())
                .build()
        );

        FollowParameters intakeFirstSet = new FollowParameters(Constants.FORWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_PRELOADS, FIRST_INTAKE_START))
                .setLinearHeadingInterpolation(SHOOT_PRELOADS.getHeading(), FIRST_INTAKE_END.getHeading(), 0.8)
                .addPath(ColoredDecodePose.makeBezier(FIRST_INTAKE_START, FIRST_INTAKE_END))
                .setConstantHeadingInterpolation(FIRST_INTAKE_END.getHeading())
                .build()
        );

        FollowParameters shootFirstSet = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_INTAKE_END, FIRST_SHOOT))
                .setConstantHeadingInterpolation(FIRST_SHOOT.getHeading())
                .build()
        );

        FollowParameters intakeSecondSet = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_SHOOT, SECOND_INTAKE_START))
                .setLinearHeadingInterpolation(FIRST_SHOOT.getHeading(), SECOND_INTAKE_END.getHeading(), 0.8)
                .addPath(ColoredDecodePose.makeBezier(SECOND_INTAKE_ACTUAL_START, SECOND_INTAKE_END))
                .setConstantHeadingInterpolation(SECOND_INTAKE_END.getHeading())
                .build()
        );

        FollowParameters shootSecondSet = new FollowParameters(Constants.FORWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_INTAKE_END, SECOND_SHOOT))
                .setConstantHeadingInterpolation(SECOND_SHOOT.getHeading())
                .build()
        );

        FollowParameters intakeThirdSet = new FollowParameters(Constants.FORWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_SHOOT, THIRD_INTAKE_START))
                .setLinearHeadingInterpolation(SECOND_SHOOT.getHeading(), THIRD_INTAKE_END.getHeading(), 0.8)
                .addPath(ColoredDecodePose.makeBezier(THIRD_INTAKE_ACTUAL_START, THIRD_INTAKE_END))
                .setConstantHeadingInterpolation(THIRD_INTAKE_END.getHeading())
                .build()
        );

        FollowParameters shootThirdSet = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(THIRD_INTAKE_END, THIRD_SHOOT))
                .setConstantHeadingInterpolation(THIRD_SHOOT.getHeading())
                .build()
        );

        FollowParameters goToStart = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(THIRD_SHOOT, START_POSE))
                .setLinearHeadingInterpolation(THIRD_SHOOT.getHeading(), PARTNER_PUSH.getHeading())
                .build()
        );

        FollowParameters park = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_POSE, PARTNER_PUSH))
                .setConstantHeadingInterpolation(PARTNER_PUSH.getHeading())
                .addPath(ColoredDecodePose.makeBezier(PARTNER_PUSH, PARK))
                .setConstantHeadingInterpolation(PARTNER_PUSH.getHeading())
                .build()
        );

        FollowParameters parkNoPush = new FollowParameters(Constants.BACKWARD_PROPORTIONAL, follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, PARTNER_PUSH))
                .setConstantHeadingInterpolation(PARTNER_PUSH.getHeading())
                .build()
        );

        return List.of(initialShoot, intakeFirstSet, shootFirstSet, intakeSecondSet,
                shootSecondSet, intakeThirdSet, shootThirdSet, goToStart, park, parkNoPush);
    }
}
