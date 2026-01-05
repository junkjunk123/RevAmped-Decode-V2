package org.firstinspires.ftc.teamcode.opmodes.paths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;

import java.util.List;

@Configurable
public class PinpointTestPaths implements PathSupplier {
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(55.35, 6.914, Math.PI);
    public static ColoredDecodePose CHECK_ONE = new ColoredDecodePose(13.87573, 30.9268, Math.PI);
    public static ColoredDecodePose CHECK_TWO = new ColoredDecodePose(55.14479, 30.926, Math.PI);
    public static ColoredDecodePose SPIN_PT1 = new ColoredDecodePose(55.14479, 30.926, Math.toRadians(30));
    public static ColoredDecodePose SPIN_PT2 = new ColoredDecodePose(55.14479, 30.926, Math.toRadians(330));
    public static ColoredDecodePose SPIN_PT3 = new ColoredDecodePose(55.14479, 30.926, Math.PI);
    public static ColoredDecodePose CHECK_THREE = new ColoredDecodePose(13.875, 38.674, Math.PI);
    public static ColoredDecodePose LINEAR_ONE = new ColoredDecodePose(40.832, 49.520);
    public static ColoredDecodePose LINEAR_TWO = new ColoredDecodePose(13.87, 51.919);
    public static ColoredDecodePose TANGENTIAL_ONE = new ColoredDecodePose(55.167, 32.280);
    public static ColoredDecodePose TANGENTIAL_CONTROL_ONE = new ColoredDecodePose(51.83,89.96);
    public static ColoredDecodePose TANGENTIAL_CONTROL_TWO = new ColoredDecodePose(67.87,37.12);
    public static ColoredDecodePose RESET = new ColoredDecodePose(55.348, 6.914, Math.PI);

    @Override
    public Pose startPose() {
        return START_POSE.getPose();
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {

        FollowParameters firstLineCheck = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_POSE,CHECK_ONE))
                .setConstantHeadingInterpolation(CHECK_ONE.getHeading())
                .build()
        );

        FollowParameters secondLineCheck = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(CHECK_ONE, CHECK_TWO))
                .setConstantHeadingInterpolation(CHECK_TWO.getHeading())
                .build()
        );

        FollowParameters spin1 = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(CHECK_TWO))
                .setConstantHeadingInterpolation(SPIN_PT1.getHeading())
                .build()
        );

        FollowParameters spin2 = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(CHECK_TWO))
                .setConstantHeadingInterpolation(SPIN_PT2.getHeading())
                .build()
        );

        FollowParameters spin3 = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(CHECK_TWO))
                .setConstantHeadingInterpolation(SPIN_PT3.getHeading())
                .build()
        );

        FollowParameters thirdLineCheck = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(CHECK_TWO, CHECK_THREE))
                .setConstantHeadingInterpolation(CHECK_THREE.getHeading())
                .build()
        );

        FollowParameters firstLinearCheck = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(CHECK_THREE,LINEAR_ONE))
                .setLinearHeadingInterpolation(180,0)
                .build()
        );
        FollowParameters secondLinearCheck = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(LINEAR_ONE,LINEAR_TWO))
                .setLinearHeadingInterpolation(0,180)
                .build()
        );
        FollowParameters tangentialCheck = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(
                        LINEAR_TWO,
                        TANGENTIAL_CONTROL_ONE,
                        TANGENTIAL_CONTROL_TWO,
                        TANGENTIAL_ONE

                ))
                .setTangentHeadingInterpolation()
                .build()
        );
        FollowParameters resetPath = new FollowParameters(follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(TANGENTIAL_ONE,RESET))
                .setConstantHeadingInterpolation(RESET.getHeading())
                .build()
        );
        return List.of(firstLineCheck,secondLineCheck,spin1,spin2,spin3,thirdLineCheck,firstLinearCheck,secondLinearCheck,tangentialCheck,resetPath);
    }
}
