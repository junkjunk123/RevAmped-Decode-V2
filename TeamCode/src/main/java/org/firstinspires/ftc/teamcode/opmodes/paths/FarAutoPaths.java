package org.firstinspires.ftc.teamcode.opmodes.paths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.utils.FollowParameters;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Configurable
public class FarAutoPaths implements PathSupplier {
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(55, 7.5, -Math.PI / 2);
    public static ColoredDecodePose DETECT_OBELISK = new ColoredDecodePose(55, 60, -Math.PI / 2);
    public static ColoredDecodePose CORNER_ONE = new ColoredDecodePose(55, 23.5, -Math.PI / 2);
    public static ColoredDecodePose CORNER_TWO = new ColoredDecodePose(7, 23.5, -Math.PI / 2);
    public static ColoredDecodePose CORNER_THREE = new ColoredDecodePose(7, 23.5, -Math.PI / 2);
    public static ColoredDecodePose SHOOT_POSE = new ColoredDecodePose(55, 7.5, -Math.PI / 2);
    public static ColoredDecodePose PARK = new ColoredDecodePose(55, 55, -Math.PI / 2);

    @Override
    public Pose startPose() {
        return START_POSE.getPose();
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {

        FollowParameters shootInitial = new FollowParameters(follower.pathBuilder()
                .addPath(new BezierPoint(START_POSE))
                .setConstantHeadingInterpolation(START_POSE.getHeading())
                .build()
        );

        FollowParameters cornerOne = new FollowParameters(follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, CORNER_ONE))
                .setConstantHeadingInterpolation(CORNER_ONE.getHeading())
                .addParametricCallback(1, () -> {
                    Robot.INSTANCE.flywheel().setCurrentState(Types.FlywheelState.STOPPED);
                    Scheduler.getInstance().add(new Delay(Robot.INSTANCE.tableServo()::reached,
                            () -> Scheduler.getInstance().add(new Delay(500, () -> Robot.INSTANCE.popper().setCurrentState(Types.PopperState.NEUTRAL))))
                            .setMaxTimeToRun(1500));
                })
                .build()
        );

        FollowParameters cornerTwo = new FollowParameters(follower.pathBuilder()
                .addPath(new BezierLine(CORNER_ONE, CORNER_TWO))
                .setConstantHeadingInterpolation(CORNER_TWO.getHeading())
                .addParametricCallback(0.5, () -> Robot.INSTANCE.intakeMotor().setCurrentState(Types.IntakeState.INTAKE))
                .build()
        );

        FollowParameters intake = new FollowParameters(follower.pathBuilder()
                .addPath(new BezierLine(CORNER_TWO, CORNER_THREE))
                .setConstantHeadingInterpolation(CORNER_THREE.getHeading())
                .build()
        );

        FollowParameters shoot = new FollowParameters(follower.pathBuilder()
                .addPath(new BezierLine(CORNER_THREE, SHOOT_POSE))
                .setConstantHeadingInterpolation(SHOOT_POSE.getHeading())
                .build()
        );

        FollowParameters park = new FollowParameters(follower.pathBuilder()
                .addPath(new BezierLine(SHOOT_POSE, PARK))
                .setReversed()
                .build()
        );

        return List.of(shootInitial, cornerOne, cornerTwo, intake, shoot, park);
    }
}
