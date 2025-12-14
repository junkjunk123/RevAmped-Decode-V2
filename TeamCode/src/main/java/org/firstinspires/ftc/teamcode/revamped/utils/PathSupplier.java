package org.firstinspires.ftc.teamcode.revamped.utils;

import android.util.Pair;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import java.util.List;

public interface PathSupplier {
    Pose startPose();
    List<FollowParameters> paths(Follower follower);
}
