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
public class CloseAutoPathsMTI implements PathSupplier {
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(32, 134, Math.toRadians(90));

    public static ColoredDecodePose START_CONTROL = new ColoredDecodePose(32, 117);
    public static ColoredDecodePose START_END = new ColoredDecodePose(50, 85, Math.toRadians(120));

    public static ColoredDecodePose PRE_SPIKE_1_CONTROL = new ColoredDecodePose(56, 66);
    public static ColoredDecodePose PRE_SPIKE_1 = new ColoredDecodePose(45, 60, Math.toRadians(180));

    public static ColoredDecodePose SPIKE_1 = new ColoredDecodePose(20, 60);

    public static ColoredDecodePose SPIKE_1_SHOOT_CONTROL = new ColoredDecodePose(37, 60);
    public static ColoredDecodePose SHOOT_POSE = new ColoredDecodePose(56, 76);

    public static ColoredDecodePose PRE_GATE_CONTROL = new ColoredDecodePose(40, 60);
    public static ColoredDecodePose PRE_GATE = new ColoredDecodePose(26, 59);

    public static ColoredDecodePose GATE = new ColoredDecodePose(12.5, 58.5, Math.toRadians(155));

    public static ColoredDecodePose SPIKE_2_CONTROL = new ColoredDecodePose(45, 84);
    public static ColoredDecodePose SPIKE_2 = new ColoredDecodePose(19, 84, Math.toRadians(180));

    public static ColoredDecodePose SPIKE_2_PRE_SHOOT = new ColoredDecodePose(40, 80, Math.toRadians(170));

    public static ColoredDecodePose SPIKE_2_SHOOT = new ColoredDecodePose(56, 76, Math.toRadians(225));

    public static ColoredDecodePose PARK = new ColoredDecodePose(54, 74);

    @Override
    public Pose startPose() {
        return START_POSE.getPose();
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {

        FollowParameters start = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_POSE, START_CONTROL, START_END))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters preSpike1 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(START_END, PRE_SPIKE_1_CONTROL, PRE_SPIKE_1))
                .setLinearHeadingInterpolation(START_END.getHeading(), PRE_SPIKE_1.getHeading())
                .build()
        );

        FollowParameters spike1 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(PRE_SPIKE_1, SPIKE_1))
                .setTangentHeadingInterpolation()
                .build()
        );

        FollowParameters spike1Shoot = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SPIKE_1, SPIKE_1_SHOOT_CONTROL, SHOOT_POSE))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters preGate = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_POSE, PRE_GATE_CONTROL, PRE_GATE))
                .setTangentHeadingInterpolation()
                .build()
        );

        FollowParameters gate = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(PRE_GATE, GATE))
                .setConstantHeadingInterpolation(GATE.getHeading())
                .build()
        );

        FollowParameters gateShoot = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(GATE, SPIKE_1_SHOOT_CONTROL, SHOOT_POSE))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters spike2 = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SHOOT_POSE, SPIKE_2_CONTROL, SPIKE_2))
                .setConstantHeadingInterpolation(SPIKE_2.getHeading())
                .build()
        );

        FollowParameters spike2PreShoot = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SPIKE_2, SPIKE_2_PRE_SHOOT))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build()
        );

        FollowParameters spike2Shoot = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SPIKE_2_PRE_SHOOT, SPIKE_2_SHOOT))
                .setLinearHeadingInterpolation(SPIKE_2_PRE_SHOOT.getHeading(), SPIKE_2_SHOOT.getHeading())
                .build()
        );

        FollowParameters park = new FollowParameters(Constants.DEFAULT_PROPORTIONAL, follower.pathBuilder()
                .addPath(ColoredDecodePose.makeBezier(SPIKE_2_SHOOT, PARK))
                .setTangentHeadingInterpolation()
                .build()
        );

        return List.of(
                start,
                preSpike1,
                spike1,
                spike1Shoot,
                preGate,
                gate,
                gateShoot,
                spike2,
                spike2PreShoot,
                spike2Shoot,
                park
        );
    }
}