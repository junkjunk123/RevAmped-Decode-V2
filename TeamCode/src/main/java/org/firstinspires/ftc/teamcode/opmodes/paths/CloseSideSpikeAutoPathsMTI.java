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
public class CloseSideSpikeAutoPathsMTI implements PathSupplier {
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
    public static ColoredDecodePose FIRST_SPIKE_2_CONTROL = new ColoredDecodePose(22, 74);
    public static ColoredDecodePose FIRST_SPIKE_2 = new ColoredDecodePose(13, 74, Math.toRadians(270));

    //SHOOTING FIRST SPIKE MARK
    public static ColoredDecodePose FIRST_SPIKE_SHOOT_1 = new ColoredDecodePose(28, 79, Math.toRadians(270));
    public static ColoredDecodePose FIRST_SPIKE_SHOOT_2 = new ColoredDecodePose(56, 80, Math.toRadians(225));

    //GRABBING SECOND SPIKE MARK
    public static ColoredDecodePose SECOND_SPIKE_1_CONTROL = new ColoredDecodePose(56, 65);
    public static ColoredDecodePose SECOND_SPIKE_1 = new ColoredDecodePose(45, 59, Math.toRadians(180));
    public static ColoredDecodePose SECOND_SPIKE_2 = new ColoredDecodePose(18.5, 61, Math.toRadians(180));

    //SHOOTING SECOND SPIKE MARK
    public static ColoredDecodePose SECOND_SPIKE_SHOOT_CONTROL = new ColoredDecodePose(37, 66);
    public static ColoredDecodePose SECOND_SPIKE_SHOOT = new ColoredDecodePose(56, 80, Math.toRadians(225));

    //===GATE PATHS (SHOULD BE THE SAME FOR CYCLE SPAM)===
    //OPENING GATE
    public static ColoredDecodePose GATE_1_CONTROL = new ColoredDecodePose(43, 60);
    public static ColoredDecodePose GATE_1 = new ColoredDecodePose(33, 60);
    public static ColoredDecodePose GATE_2 = new ColoredDecodePose(12.5, 56.5, Math.toRadians(152));

    //SHOOTING FROM GATE CYCLE
    public static ColoredDecodePose GATE_SHOOT = new ColoredDecodePose(56, 80, Math.toRadians(225));
    public static ColoredDecodePose GATE_SHOOT_CONTROL = new ColoredDecodePose(37, 64);



    //PARK
    public static ColoredDecodePose PARK = new ColoredDecodePose(54, 76);

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

        FollowParameters intakeSpike1 = new FollowParameters(Constants.CONSERVATIVE_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(PRELOADS_SHOOT, FIRST_SPIKE_1_CONTROL, FIRST_SPIKE_1))
                .setConstantHeadingInterpolation(PRELOADS_SHOOT.getHeading())
                .addPath(ColoredDecodePose.makeBezier(FIRST_SPIKE_1, FIRST_SPIKE_2_CONTROL, FIRST_SPIKE_2))
                .setConstantHeadingInterpolation(FIRST_SPIKE_2.getHeading())
                .build()
        );

        FollowParameters shootSpike1 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_SPIKE_2, FIRST_SPIKE_SHOOT_1))
                .setConstantHeadingInterpolation(FIRST_SPIKE_2.getHeading())
                .addPath(ColoredDecodePose.makeBezier(FIRST_SPIKE_SHOOT_1, FIRST_SPIKE_SHOOT_2))
                .setLinearHeadingInterpolation(FIRST_SPIKE_SHOOT_1.getHeading(), FIRST_SPIKE_SHOOT_2.getHeading())
                .build()
        );

        FollowParameters intakeSpike2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(FIRST_SPIKE_SHOOT_2, SECOND_SPIKE_1_CONTROL, SECOND_SPIKE_1))
                .setLinearHeadingInterpolation(FIRST_SPIKE_SHOOT_2.getHeading(), SECOND_SPIKE_1.getHeading())
                .addPath(ColoredDecodePose.makeBezier(SECOND_SPIKE_1, SECOND_SPIKE_2))
                .setConstantHeadingInterpolation(SECOND_SPIKE_1.getHeading())
                .build()
        );

        FollowParameters shootSpike2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SECOND_SPIKE_2, SECOND_SPIKE_SHOOT_CONTROL, SECOND_SPIKE_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );
        //FIRST SPIKE SHOOT & GATE_SHOOT are the same point

        //GATE SPAM PATHS
        FollowParameters gateIntake1 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT, GATE_1_CONTROL, GATE_1))
                .setTangentHeadingInterpolation()
                .addPath(ColoredDecodePose.makeBezier(GATE_1,GATE_2))
                .setConstantHeadingInterpolation(GATE_2.getHeading())
                .build()
        );

        FollowParameters gateShoot1 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_2, GATE_SHOOT_CONTROL, GATE_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters gateIntake2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT, GATE_1_CONTROL, GATE_1))
                .setTangentHeadingInterpolation()
                .addPath(ColoredDecodePose.makeBezier(GATE_1,GATE_2))
                .setConstantHeadingInterpolation(GATE_2.getHeading())
                .build()
        );

        FollowParameters gateShoot2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_2, GATE_SHOOT_CONTROL, GATE_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters gateIntake3 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT, GATE_1_CONTROL, GATE_1))
                .setTangentHeadingInterpolation()
                .addPath(ColoredDecodePose.makeBezier(GATE_1,GATE_2))
                .setConstantHeadingInterpolation(GATE_2.getHeading())
                .build()
        );

        FollowParameters gateShoot3 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_2, GATE_SHOOT_CONTROL, GATE_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters gateIntake4 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT, GATE_1_CONTROL, GATE_1))
                .setTangentHeadingInterpolation()
                .addPath(ColoredDecodePose.makeBezier(GATE_1,GATE_2))
                .setConstantHeadingInterpolation(GATE_2.getHeading())
                .build()
        );

        FollowParameters gateShoot4 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_2, GATE_SHOOT_CONTROL, GATE_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters gateIntake5 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT, GATE_1_CONTROL, GATE_1))
                .setTangentHeadingInterpolation()
                .addPath(ColoredDecodePose.makeBezier(GATE_1,GATE_2))
                .setConstantHeadingInterpolation(GATE_2.getHeading())
                .build()
        );

        FollowParameters gateShoot5 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_2, GATE_SHOOT_CONTROL, GATE_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );


        FollowParameters park = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE_SHOOT, PARK))
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
            park

        );
    }
}