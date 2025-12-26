package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.utils.FollowParameters;

import java.util.List;

public interface PathSupplier {
    Pose startPose();
    List<FollowParameters> paths(Follower follower);
}
