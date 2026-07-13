package org.firstinspires.ftc.teamcode.opmodes.paths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;

import java.util.List;

@Configurable
public class RedClose27AutoPathsMTI implements PathSupplier {
    /*
    NAMING CONVENTION:
    <NAME>_<SUB-PATH NUMBER IF ANY>_<CONTROL/(NONE IF NOT CONTROL)>
     */
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(32, 134, Math.toRadians(270));

    //SHOOTING PRELOADS
    public static ColoredDecodePose PRELOADS_SHOOT = new ColoredDecodePose(32, 98, Math.toRadians(270));

    //GRABBING FIRST SPIKE MARK
    public static ColoredDecodePose FIRST_SPIKE_1_CONTROL = new ColoredDecodePose(24, 98);
    public static ColoredDecodePose FIRST_SPIKE_1 = new ColoredDecodePose(24, 94, Math.toRadians(270));
    public static ColoredDecodePose FIRST_SPIKE_2 = new ColoredDecodePose(24, 87, Math.toRadians(270));

    //SHOOTING FIRST SPIKE MARK
    public static ColoredDecodePose FIRST_SPIKE_SHOOT = new ColoredDecodePose(46, 84, Math.toRadians(270));

    //GRABBING SECOND SPIKE MARK
    public static ColoredDecodePose SECOND_SPIKE_1 = new ColoredDecodePose(46, 53, Math.toRadians(190));
    public static ColoredDecodePose SECOND_SPIKE_2 = new ColoredDecodePose(40, 58, Math.toRadians(180));
    public static ColoredDecodePose SECOND_SPIKE_3 = new ColoredDecodePose(18.5, 61, Math.toRadians(180));

    //SHOOTING SECOND SPIKE MARK
    public static ColoredDecodePose SECOND_SPIKE_SHOOT_CONTROL = new ColoredDecodePose(37, 66);
    public static ColoredDecodePose SECOND_SPIKE_SHOOT = new ColoredDecodePose(56, 80, Math.toRadians(209));

    //===GATE PATHS (SHOULD BE THE SAME FOR CYCLE SPAM)===
    //OPENING GATE
    public static ColoredDecodePose GATE_1 = new ColoredDecodePose(39, 70,Math.toRadians(209));
    public static ColoredDecodePose GATE_2 = new ColoredDecodePose(29,63,Math.toRadians(152));
    public static ColoredDecodePose GATE_3 = new ColoredDecodePose(10.25, 58, Math.toRadians(152));
    //SHOOTING FROM GATE CYCLE
    public static ColoredDecodePose GATE_SHOOT_1 = new ColoredDecodePose(36, 59, Math.toRadians(209));
    public static ColoredDecodePose GATE_SHOOT_2 = new ColoredDecodePose(58, 79, Math.toRadians(209));



    //PARK
    public static ColoredDecodePose PARK = new ColoredDecodePose(54, 76);

    @Override
    public Pose startPose() {
        return START_POSE.getPose();
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {

        FollowParameters shootPreloads = new FollowParameters(Constants.AGGRESSIVE_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_POSE, PRELOADS_SHOOT))
                .setConstantHeadingInterpolation(PRELOADS_SHOOT.getHeading())
                .build()
        );

        FollowParameters intakeSpike1 = new FollowParameters(Constants.CONSERVATIVE_PROPORTIONAL, follower.pathBuilder()
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
        //FIRST SPIKE SHOOT & GATE_SHOOT are the same point

        //GATE SPAM PATHS
        FollowParameters gateIntake1 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.5, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT_2, GATE_1))
                .setTangentHeadingInterpolation()
                .addPath(ColoredDecodePose.makeBezier(GATE_1,GATE_2))
                .setLinearHeadingInterpolation(GATE_1.getHeading(),GATE_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(GATE_2,GATE_3))
                .setConstantHeadingInterpolation(GATE_3.getHeading())
                .build()
        );

        FollowParameters gateShoot1 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.75, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_3, GATE_SHOOT_1, GATE_SHOOT_2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters gateIntake2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.5, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT_2, GATE_1))
                .setTangentHeadingInterpolation()
                .addPath(ColoredDecodePose.makeBezier(GATE_1,GATE_2))
                .setLinearHeadingInterpolation(GATE_1.getHeading(),GATE_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(GATE_2,GATE_3))
                .setConstantHeadingInterpolation(GATE_3.getHeading())
                .build()
        );

        FollowParameters gateShoot2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.75, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_3, GATE_SHOOT_1, GATE_SHOOT_2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters gateIntake3 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.5, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT_2, GATE_1))
                .setTangentHeadingInterpolation()
                .addPath(ColoredDecodePose.makeBezier(GATE_1,GATE_2))
                .setLinearHeadingInterpolation(GATE_1.getHeading(),GATE_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(GATE_2,GATE_3))
                .setConstantHeadingInterpolation(GATE_3.getHeading())
                .build()
        );

        FollowParameters gateShoot3 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.75, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_3, GATE_SHOOT_1, GATE_SHOOT_2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters gateIntake4 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.5, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT_2, GATE_1))
                .setTangentHeadingInterpolation()
                .addPath(ColoredDecodePose.makeBezier(GATE_1,GATE_2))
                .setLinearHeadingInterpolation(GATE_1.getHeading(),GATE_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(GATE_2,GATE_3))
                .setConstantHeadingInterpolation(GATE_3.getHeading())
                .build()
        );

        FollowParameters gateShoot4 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.75, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_3, GATE_SHOOT_1, GATE_SHOOT_2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters gateIntake5 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.5, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT_2, GATE_1))
                .setTangentHeadingInterpolation()
                .addPath(ColoredDecodePose.makeBezier(GATE_1,GATE_2))
                .setLinearHeadingInterpolation(GATE_1.getHeading(),GATE_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(GATE_2,GATE_3))
                .setConstantHeadingInterpolation(GATE_3.getHeading())
                .build()
        );

        FollowParameters gateShoot5 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.75, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_3, GATE_SHOOT_1, GATE_SHOOT_2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters gateIntake6 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.5, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT_2, GATE_1))
                .setTangentHeadingInterpolation()
                .addPath(ColoredDecodePose.makeBezier(GATE_1,GATE_2))
                .setLinearHeadingInterpolation(GATE_1.getHeading(),GATE_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(GATE_2,GATE_3))
                .setConstantHeadingInterpolation(GATE_3.getHeading())
                .build()
        );

        FollowParameters gateShoot6 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, 0.75, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_3, GATE_SHOOT_1, GATE_SHOOT_2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );


        FollowParameters park = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT_2, PARK))
                .setTangentHeadingInterpolation()
                .build()
        );

//        FollowParameters spike1 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
//                .addPath(ColoredDecodePose.makeBezier(PRE_SPIKE_1, SPIKE_1))
//                .setTangentHeadingInterpolation()
//                .build()
//        );
//
//        FollowParameters spike1Shoot = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
//                .addPath(ColoredDecodePose.makeBezier(SPIKE_1, SPIKE_1_SHOOT_CONTROL, SHOOT_POSE))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build()
//        );
//
//        FollowParameters preGate = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
//                .addPath(ColoredDecodePose.makeBezier(SHOOT_POSE, PRE_GATE_CONTROL, PRE_GATE))
//                .setTangentHeadingInterpolation()
//                .build()
//        );
//
//        FollowParameters gate = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
//                .addPath(ColoredDecodePose.makeBezier(PRE_GATE, GATE))
//                .setConstantHeadingInterpolation(GATE.getHeading())
//                .build()
//        );
//
//        FollowParameters gateShoot = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
//                .addPath(ColoredDecodePose.makeBezier(GATE, SPIKE_1_SHOOT_CONTROL, SHOOT_POSE))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build()
//        );
//
//        FollowParameters spike2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
//                .addPath(ColoredDecodePose.makeBezier(SHOOT_POSE, SPIKE_2_CONTROL, SPIKE_2))
//                .setConstantHeadingInterpolation(SPIKE_2.getHeading())
//                .build()
//        );
//
//        FollowParameters spike2PreShoot = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
//                .addPath(ColoredDecodePose.makeBezier(SPIKE_2, SPIKE_2_PRE_SHOOT))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build()
//        );
//
//        FollowParameters spike2Shoot = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
//                .addPath(ColoredDecodePose.makeBezier(SPIKE_2_PRE_SHOOT, SPIKE_2_SHOOT))
//                .setLinearHeadingInterpolation(SPIKE_2_PRE_SHOOT.getHeading(), SPIKE_2_SHOOT.getHeading())
//                .build()
//        );
//
//        FollowParameters park = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
//                .addPath(ColoredDecodePose.makeBezier(SPIKE_2_SHOOT, PARK))
//                .setTangentHeadingInterpolation()
//                .build()
//        );

        return List.of(
            shootPreloads,
            intakeSpike1,
            shootSpike1,
            intakeSpike2,
            shootSpike2,
            gateIntake1,
            gateShoot1,
            gateIntake2,
            gateShoot2,
            gateIntake3,
            gateShoot3,
            gateIntake4,
            gateShoot4,
            gateIntake5,
            gateShoot5,
            gateIntake6,
            gateShoot6,
            park

        );
    }
}