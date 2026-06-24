package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.groups.Sequential;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;

public record FollowParameters(PathChain pathChain, boolean holdEnd, double maxPower, double kP, double brakingStrength) {
    public FollowParameters(PathChain pathChain, boolean holdEnd, double maxPower, double kP) {
        this(pathChain, holdEnd, maxPower, Constants.DEFAULT_PROPORTIONAL, kP);
    }

    public FollowParameters(PathChain pathChain, boolean holdEnd) {
        this(pathChain, holdEnd, 1.0, Constants.DEFAULT_PROPORTIONAL);
    }

    public FollowParameters(PathChain pathChain, double maxPower) {
        this(pathChain, true, maxPower, Constants.DEFAULT_PROPORTIONAL);
    }

    public FollowParameters(double kP, PathChain pathChain) {
        this(pathChain, true, 1.0, kP);
    }

    public FollowParameters(double kP, double brakingStrength, PathChain pathChain) {
        this(pathChain, true, 1.0, kP, brakingStrength);
    }

    public FollowParameters(PathChain pathChain) {
        this(pathChain,true, 1.0, Constants.DEFAULT_PROPORTIONAL);
    }

    public void follow(Follower follower) {
        follow(follower, brakingStrength);
    }

    public void follow(Follower follower, double brakingStrength) {
        brakingStrength = 1 / brakingStrength;
        follower.vectorCalculator.predictiveBrakingController.setCoefficients(
                new PredictiveBrakingCoefficients(kP, Constants.K_LINEAR_BRAKE * brakingStrength, Constants.K_QUADRATIC_BRAKE * brakingStrength));
        follower.followPath(pathChain, maxPower, holdEnd);
    }

    public Command followCommand(Drivetrain drivetrain) {
        return new Command()
                .setStart(() -> follow(drivetrain.follower))
                .setDone(() -> drivetrain.velocityCondition(4));
    }
}
