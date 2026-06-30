package org.firstinspires.ftc.teamcode.opmodes.paths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.commands.follow.Follow;

import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;

import java.util.List;

import kotlin.math.UMathKt;

@Configurable
public class FarAutoPathsMTI implements PathSupplier {
    /*
    NAMING CONVENTION:
    <NAME>_<SUB-PATH NUMBER IF ANY>_<CONTROL/(NONE IF NOT CONTROL)>
     */
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(42, 9, Math.toRadians(90));


    public static ColoredDecodePose FIRST_SPIKE = new ColoredDecodePose(23.5, 30, Math.toRadians(90));

    public static ColoredDecodePose FIRST_SPIKE_CONTROL = new ColoredDecodePose(23.5, 16);

    public static ColoredDecodePose FIRST_SHOOT = new ColoredDecodePose(45, 10.5);

    public static ColoredDecodePose FIRST_SHOOT_CONTROL = new ColoredDecodePose(23.5, 16);

    public static ColoredDecodePose SECOND_SPIKE = new ColoredDecodePose(13.5, 12, Math.toRadians(180));

    public static ColoredDecodePose SECOND_SHOOT = new ColoredDecodePose(46.5, 13);

    public static ColoredDecodePose SWEEP_1 = new ColoredDecodePose(13.5, 8, Math.toRadians(180));

    public static ColoredDecodePose SWEEP_2 = new ColoredDecodePose(16, 14.5, Math.toRadians(125));

    public static ColoredDecodePose SWEEP_2_CONTROL = new ColoredDecodePose(16, 14.5);

    public static ColoredDecodePose SWEEP_3 = new ColoredDecodePose(14, 34.5, Math.toRadians(125));

    public static ColoredDecodePose SWEEP_SHOOT = new ColoredDecodePose(57.5,17.5);

    public static ColoredDecodePose PARK = new ColoredDecodePose(49.5, 20.5);



    @Override
    public Pose startPose() {
        return START_POSE.getPose();
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {
        /*
        FollowParameters <path name> = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
            <add paths here>
            .build()

        to add a path:
            .addPath(ColoredDecodePose.makeBezier(<start>,<controls...>,<end>))
            .<heading interpolation>
            .<reversed if needed>
         */

        //spike 1 cycle
        FollowParameters intakeSpike1 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_POSE, FIRST_SPIKE_CONTROL, FIRST_SPIKE))
                .setConstantHeadingInterpolation(FIRST_SPIKE.getHeading())
                .build()
        );

        FollowParameters shootSpike1 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_SPIKE, FIRST_SHOOT_CONTROL, FIRST_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        //spike 2 cycle
        FollowParameters intakeSpike2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_SHOOT, SECOND_SPIKE))
                .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                .build()
        );

        FollowParameters shootSpike2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_SPIKE, SECOND_SHOOT))
                .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                .build()
        );

        //spamsweeppath
        FollowParameters intakeSweepHP = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_SHOOT, SWEEP_1))
                .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                .build()
        );

        FollowParameters hpToShoot = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SWEEP_1, SECOND_SHOOT))
                .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                .build()
        );

        FollowParameters sweepAndShoot = new FollowParameters(Constants.CONSERVATIVE_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SWEEP_1, SWEEP_2_CONTROL, SWEEP_2))
                .setLinearHeadingInterpolation(SWEEP_1.getHeading(), SWEEP_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(SWEEP_2, SWEEP_3))
                .setConstantHeadingInterpolation(SWEEP_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(SWEEP_3, SWEEP_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters intakeSweepHP2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_SHOOT, SWEEP_1))
                .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                .build()
        );

        FollowParameters hpToShoot2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SWEEP_1, SECOND_SHOOT))
                .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                .build()
        );

        FollowParameters sweepAndShoot2 = new FollowParameters(Constants.CONSERVATIVE_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SWEEP_1, SWEEP_2_CONTROL, SWEEP_2))
                .setLinearHeadingInterpolation(SWEEP_1.getHeading(), SWEEP_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(SWEEP_2, SWEEP_3))
                .setConstantHeadingInterpolation(SWEEP_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(SWEEP_3, SWEEP_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );
        FollowParameters intakeSweepHP3 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_SHOOT, SWEEP_1))
                .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                .build()
        );

        FollowParameters hpToShoot3 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SWEEP_1, SECOND_SHOOT))
                .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                .build()
        );

        FollowParameters sweepAndShoot3 = new FollowParameters(Constants.CONSERVATIVE_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SWEEP_1, SWEEP_2_CONTROL, SWEEP_2))
                .setLinearHeadingInterpolation(SWEEP_1.getHeading(), SWEEP_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(SWEEP_2, SWEEP_3))
                .setConstantHeadingInterpolation(SWEEP_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(SWEEP_3, SWEEP_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );
        FollowParameters intakeSweepHP4 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_SHOOT, SWEEP_1))
                .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                .build()
        );

        FollowParameters hpToShoot4 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SWEEP_1, SECOND_SHOOT))
                .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                .build()
        );

        FollowParameters sweepAndShoot4 = new FollowParameters(Constants.CONSERVATIVE_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SWEEP_1, SWEEP_2_CONTROL, SWEEP_2))
                .setLinearHeadingInterpolation(SWEEP_1.getHeading(), SWEEP_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(SWEEP_2, SWEEP_3))
                .setConstantHeadingInterpolation(SWEEP_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(SWEEP_3, SWEEP_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );
        FollowParameters intakeSweepHP5 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_SHOOT, SWEEP_1))
                .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                .build()
        );

        FollowParameters hpToShoot5 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SWEEP_1, SECOND_SHOOT))
                .setConstantHeadingInterpolation(SECOND_SPIKE.getHeading())
                .build()
        );

        FollowParameters sweepAndShoot5 = new FollowParameters(Constants.CONSERVATIVE_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SWEEP_1, SWEEP_2_CONTROL, SWEEP_2))
                .setLinearHeadingInterpolation(SWEEP_1.getHeading(), SWEEP_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(SWEEP_2, SWEEP_3))
                .setConstantHeadingInterpolation(SWEEP_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(SWEEP_3, SWEEP_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );


        //park path
        FollowParameters park = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SWEEP_SHOOT, PARK))
                .setTangentHeadingInterpolation()
                .build()
        );


        return List.of(
            intakeSpike1,
            shootSpike1,
            intakeSpike2,
            shootSpike2,
            intakeSweepHP,
            hpToShoot,
            sweepAndShoot,
            intakeSweepHP2,
            hpToShoot2,
            sweepAndShoot2,
            intakeSweepHP3,
            hpToShoot3,
            sweepAndShoot3,
            intakeSweepHP4,
            hpToShoot4,
            sweepAndShoot4,
            intakeSweepHP5,
            hpToShoot5,
            sweepAndShoot5,
            park

        );
    }
}