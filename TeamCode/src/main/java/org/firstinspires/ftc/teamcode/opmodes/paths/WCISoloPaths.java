package org.firstinspires.ftc.teamcode.opmodes.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;

import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;

import java.util.List;
import java.util.function.Supplier;

public class WCISoloPaths implements PathSupplier {
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(32, 134, Math.toRadians(270));

    public static ColoredDecodePose PRELOADS_SHOOT = new ColoredDecodePose(32, 98, Math.toRadians(270));

    public static ColoredDecodePose FIRST_SPIKE_1_CONTROL = new ColoredDecodePose(24, 98);
    public static ColoredDecodePose FIRST_SPIKE_1 = new ColoredDecodePose(24, 94, Math.toRadians(270));
    public static ColoredDecodePose FIRST_SPIKE_2 = new ColoredDecodePose(24, 87, Math.toRadians(270));

    public static ColoredDecodePose FIRST_SPIKE_SHOOT = new ColoredDecodePose(46, 84, Math.toRadians(270));

    public static ColoredDecodePose SECOND_SPIKE_1 = new ColoredDecodePose(46, 53, Math.toRadians(190));
    public static ColoredDecodePose SECOND_SPIKE_2 = new ColoredDecodePose(40, 58, Math.toRadians(180));
    public static ColoredDecodePose SECOND_SPIKE_3 = new ColoredDecodePose(18.5, 57, Math.toRadians(180));
    public static ColoredDecodePose SECOND_SPIKE_3_VIRTUAL = new ColoredDecodePose(28.5, 57, Math.toRadians(180));

    public static ColoredDecodePose SECOND_SPIKE_SHOOT_CONTROL = new ColoredDecodePose(37, 66);
    public static ColoredDecodePose SECOND_SPIKE_SHOOT = new ColoredDecodePose(56, 80, Math.toRadians(209));

    public static ColoredDecodePose GATE_1 = new ColoredDecodePose(39, 70,Math.toRadians(209));
    public static ColoredDecodePose GATE_2 = new ColoredDecodePose(29,63,Math.toRadians(151));
    public static ColoredDecodePose GATE_3 = new ColoredDecodePose(11.5, 57.5, Math.toRadians(151));

    public static ColoredDecodePose GATE_SHOOT_1 = new ColoredDecodePose(36, 59, Math.toRadians(209));
    public static ColoredDecodePose GATE_SHOOT_2 = new ColoredDecodePose(56, 79, Math.toRadians(209));

    public static ColoredDecodePose THIRD_SPIKE_CONTROL = new ColoredDecodePose(24,61);
    public static ColoredDecodePose THIRD_SPIKE = new ColoredDecodePose(24, 40, Math.toRadians(270));
    public static ColoredDecodePose THIRD_SPIKE_VIRTUAL = new ColoredDecodePose(24, 40, Math.toRadians(270));

    public static ColoredDecodePose HUMAN_PLAYER_PHYSICAL = new ColoredDecodePose(30, 13, Math.toRadians(250));
    public static ColoredDecodePose HUMAN_PLAYER_1 = new ColoredDecodePose(33, 22, Math.toRadians(250));
    public static ColoredDecodePose HUMAN_PLAYER_2 = new ColoredDecodePose(12,13,Math.toRadians(230));

    public static ColoredDecodePose HUMAN_PLAYER_FINAL_SHOOT_CONTROL = new ColoredDecodePose(30, 16);
    public static ColoredDecodePose HUMAN_PLAYER_FINAL_SHOOT = new ColoredDecodePose(41, 9, 0);

    public static ColoredDecodePose PARK = new ColoredDecodePose(36, 9);
    public static ColoredDecodePose PARTNER_INTAKE = new ColoredDecodePose(44, 9);

    public WCISoloPaths(double partnerLength) {
        //PARTNER_INTAKE = new ColoredDecodePose(64, partnerLength);
    }

    @Override
    public Pose startPose() {
        return START_POSE.getPose();
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {
        FollowParameters shootPreloads = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_POSE, PRELOADS_SHOOT))
                .setConstantHeadingInterpolation(PRELOADS_SHOOT.getHeading())
                .build()
        );

        FollowParameters intakeSpike1 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(PRELOADS_SHOOT, FIRST_SPIKE_1_CONTROL, FIRST_SPIKE_1))
                .setConstantHeadingInterpolation(PRELOADS_SHOOT.getHeading())
                .addPath(ColoredDecodePose.makeBezier(FIRST_SPIKE_1, FIRST_SPIKE_2))
                .setConstantHeadingInterpolation(FIRST_SPIKE_2.getHeading())
                .build()
        );

        FollowParameters shootSpike1 = new FollowParameters(Constants.AGGRESSIVE_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_SPIKE_2, FIRST_SPIKE_SHOOT))
                .setConstantHeadingInterpolation(FIRST_SPIKE_2.getHeading())
                .build()
        );

        FollowParameters intakeSpike2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.6, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_SPIKE_SHOOT, SECOND_SPIKE_1, SECOND_SPIKE_2, SECOND_SPIKE_3))
                .setLinearHeadingInterpolation(FIRST_SPIKE_SHOOT.getHeading(), SECOND_SPIKE_2.getHeading(), 0.25)
                .build()
        );

        FollowParameters shootSpike2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.75, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_SPIKE_3, SECOND_SPIKE_SHOOT_CONTROL, SECOND_SPIKE_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters spike2 = new FollowParameters(Constants.AGGRESSIVE_PROPORTIONAL, 0.5, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_SPIKE_SHOOT, SECOND_SPIKE_1, SECOND_SPIKE_2, SECOND_SPIKE_3_VIRTUAL))
                .setLinearHeadingInterpolation(FIRST_SPIKE_SHOOT.getHeading(), SECOND_SPIKE_2.getHeading(), 0.25)
                .addPath(ColoredDecodePose.makeBezier(SECOND_SPIKE_3, SECOND_SPIKE_SHOOT_CONTROL, SECOND_SPIKE_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        Supplier<FollowParameters> gateIntake = () -> new FollowParameters(Constants.AGGRESSIVE_PROPORTIONAL, 0.5, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT_2, GATE_1))
                .setTangentHeadingInterpolation()
                .addPath(ColoredDecodePose.makeBezier(GATE_1,GATE_2))
                .setLinearHeadingInterpolation(GATE_1.getHeading(),GATE_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(GATE_2,GATE_3))
                .setConstantHeadingInterpolation(GATE_3.getHeading())
                .build()
        );

        Supplier<FollowParameters> gateShoot = () -> new FollowParameters(Constants.AGGRESSIVE_PROPORTIONAL, 0.5, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_3, GATE_SHOOT_1, GATE_SHOOT_2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        Supplier<FollowParameters> cornerFinal = () -> new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT_2, HUMAN_PLAYER_1))
                .setConstantHeadingInterpolation(HUMAN_PLAYER_1.getHeading())
                .addPath(ColoredDecodePose.makeBezier(HUMAN_PLAYER_1,HUMAN_PLAYER_2))
                .setLinearHeadingInterpolation(HUMAN_PLAYER_1.getHeading(),HUMAN_PLAYER_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(HUMAN_PLAYER_PHYSICAL, HUMAN_PLAYER_FINAL_SHOOT_CONTROL, HUMAN_PLAYER_FINAL_SHOOT))
                .setLinearHeadingInterpolation(HUMAN_PLAYER_PHYSICAL.getHeading(), HUMAN_PLAYER_FINAL_SHOOT.getHeading())
                .build()
        );

        BezierCurve thirdSpikeReturn = ColoredDecodePose.makeBezier(THIRD_SPIKE, GATE_SHOOT_2);
        FollowParameters thirdSpike = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT_2, THIRD_SPIKE_CONTROL, THIRD_SPIKE_VIRTUAL))
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(0, 0.6, HeadingInterpolator.tangent),
                                new HeadingInterpolator.PiecewiseNode(0.6, 1.0, HeadingInterpolator.constant(THIRD_SPIKE.getHeading()))
                        )
                )
                .addPath(thirdSpikeReturn)
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        Supplier<FollowParameters> partnerIntake = () -> new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(HUMAN_PLAYER_FINAL_SHOOT, PARTNER_INTAKE))
                .setConstantHeadingInterpolation(HUMAN_PLAYER_FINAL_SHOOT.getHeading())
                .build()
        );

        FollowParameters park = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(PARTNER_INTAKE, PARK))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        return List.of(
                shootPreloads,
                intakeSpike1,
                shootSpike1,
                spike2,
                gateIntake.get(),
                gateShoot.get(),
                thirdSpike,
                gateIntake.get(),
                gateShoot.get(),
                gateIntake.get(),
                gateShoot.get(),
                cornerFinal.get(),
                partnerIntake.get(),
                park
        );
    }
}
