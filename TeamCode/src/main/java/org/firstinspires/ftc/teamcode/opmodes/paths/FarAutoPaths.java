package org.firstinspires.ftc.teamcode.opmodes.paths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

@Configurable
public class FarAutoPaths implements PathSupplier {
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(58, 8.75, Math.PI);
    public static ColoredDecodePose PRELOAD_SHOOT = new ColoredDecodePose(58, 20, Math.toRadians(150));
    public static ColoredDecodePose INTAKE_FIRST_SET = new ColoredDecodePose(15, 36);
    public static ColoredDecodePose FIRST_INTAKE_CONTROL = new ColoredDecodePose(45, 36);
    public static ColoredDecodePose SHOOT_FIRST_SET = new ColoredDecodePose(58, 20);
    public static ColoredDecodePose INTAKE_SECOND_SET = new ColoredDecodePose(10, 9);
    public static ColoredDecodePose SECOND_INTAKE_CONTROL = new ColoredDecodePose(25, 9);
    public static ColoredDecodePose SHOOT_SECOND_SET = new ColoredDecodePose(50, 9);
    public static ColoredDecodePose INTAKE_NEAR = new ColoredDecodePose(11, 36);
    public static ColoredDecodePose INTAKE_NEAR_CONTROL = new ColoredDecodePose(36, 32);
    public static ColoredDecodePose INTAKE_NEAR_CONTROL_2 = new ColoredDecodePose(36, 35);
    public static ColoredDecodePose SHOOT = new ColoredDecodePose(52, 16);
    public static ColoredDecodePose SHOOT_NEAR_CONTROL = new ColoredDecodePose(36, 16);
    public static ColoredDecodePose INTAKE_MIDDLE = new ColoredDecodePose(10, 16);
    public static ColoredDecodePose INTAKE_FAR = new ColoredDecodePose(10, 11);
    public static ColoredDecodePose INTAKE_FAR_CONTROL = new ColoredDecodePose(32, 11);
    public static ColoredDecodePose SHOOT_FAR_CONTROL = new ColoredDecodePose(43, 18);
    public static ColoredDecodePose PARK = new ColoredDecodePose(38, 11);

    @Override
    public Pose startPose() {
        return START_POSE.getPose();
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {
        FollowParameters shootPreloads = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_POSE, PRELOAD_SHOOT))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), PRELOAD_SHOOT.getHeading())
                .build()
        );

        FollowParameters intakeFirstSet = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(PRELOAD_SHOOT, FIRST_INTAKE_CONTROL, INTAKE_FIRST_SET))
                .build()
        );

        FollowParameters shootFirstSet = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(INTAKE_FIRST_SET, SHOOT_FIRST_SET))
                .setReversed()
                .build()
        );

        FollowParameters intakeSecondSet = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_FIRST_SET, SECOND_INTAKE_CONTROL, INTAKE_SECOND_SET))
                .build()
        );

        FollowParameters shootSecondSet = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(INTAKE_FIRST_SET, SHOOT_SECOND_SET))
                .setReversed()
                .build()
        );

        FollowParameters intakeNearOne = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_SECOND_SET, INTAKE_NEAR_CONTROL, INTAKE_NEAR))
                .build()
        );

        FollowParameters intakeNear = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT, INTAKE_NEAR_CONTROL_2, INTAKE_NEAR))
                .build()
        );

        Supplier<FollowParameters> shootNear = () -> new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(INTAKE_NEAR, SHOOT_NEAR_CONTROL, SHOOT))
                .setReversed()
                .build()
        );

        Supplier<FollowParameters> intakeMiddle = () -> new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT, INTAKE_MIDDLE))
                .build()
        );

        Supplier<FollowParameters> shootMiddle = () -> new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(INTAKE_MIDDLE, SHOOT))
                .setReversed()
                .build()
        );

        Supplier<FollowParameters> intakeFar = () -> new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT, INTAKE_FAR_CONTROL, INTAKE_FAR))
                .build()
        );

        Supplier<FollowParameters> shootFar = () -> new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(INTAKE_FAR, SHOOT_FAR_CONTROL, SHOOT))
                .setReversed()
                .build()
        );

        FollowParameters park = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, PARK))
                .setHeadingInterpolation(HeadingInterpolator.lazy(() -> HeadingInterpolator.constant(follower.getHeading())))
                .build()
        );

        Supplier<List<FollowParameters>> cycle = () -> List.of(
                intakeNear, shootNear.get(),
                intakeMiddle.get(), shootMiddle.get(),
                intakeFar.get(), shootFar.get()
        );

        ArrayList<FollowParameters> paths = new ArrayList<>(312);

        paths.addAll(List.of(
                shootPreloads, intakeFirstSet, shootFirstSet,
                intakeSecondSet, shootSecondSet,
                intakeNearOne, shootNear.get(),
                intakeMiddle.get(), shootMiddle.get(),
                intakeFar.get(), shootFar.get()
        ));

        for (int i = 0; i < 50; i++) {
            paths.addAll(cycle.get());
        }

        paths.add(park);

        return paths;
    }
}
