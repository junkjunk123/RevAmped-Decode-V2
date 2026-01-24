package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public record FollowParameters(PathChain pathChain, boolean holdEnd, double maxPower, double kP) {
    public FollowParameters(PathChain pathChain, boolean holdEnd) {
        this(pathChain, holdEnd, 1.0, Constants.DEFAULT_PROPORTIONAL);
    }

    public FollowParameters(PathChain pathChain, double maxPower) {
        this(pathChain, true, maxPower, Constants.DEFAULT_PROPORTIONAL);
    }

    public FollowParameters(double kP, PathChain pathChain) {
        this(pathChain, true, 1.0, kP);
    }

    public FollowParameters(PathChain pathChain) {
        this(pathChain,true, 1.0, Constants.DEFAULT_PROPORTIONAL);
    }

    public void follow(Follower follower) {
        follower.vectorCalculator.predictiveBrakingController.setCoefficients(
                new PredictiveBrakingCoefficients(kP, 0.090, 0.00125));
        follower.followPath(pathChain, maxPower, holdEnd);
    }
}
