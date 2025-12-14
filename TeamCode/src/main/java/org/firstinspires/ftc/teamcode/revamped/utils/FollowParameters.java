package org.firstinspires.ftc.teamcode.revamped.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public record FollowParameters(PathChain pathChain, boolean holdEnd, double maxPower) {
    public FollowParameters(PathChain pathChain, boolean holdEnd) {
        this(pathChain, holdEnd, 1.0);
    }

    public FollowParameters(PathChain pathChain, double maxPower) {
        this(pathChain, true, maxPower);
    }

    public FollowParameters(PathChain pathChain) {
        this(pathChain,true, 1.0);
    }

    public void follow(Follower follower) {
        follower.followPath(pathChain, maxPower, holdEnd);
    }
}
