package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import java.util.List;

public interface PathSupplier {
    Pose startPose();
    List<FollowParameters> paths(Follower follower);
}
