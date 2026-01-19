package org.firstinspires.ftc.teamcode.opmodes.paths;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;

import java.util.Collections;
import java.util.List;

@Configurable
public class UnsortedCloseAutoPaths implements PathSupplier {
    /*
    {"startPoint":{"x":15.5,"y":114,"heading":"linear","startDeg":90,"endDeg":180},"lines":[{"name":"Path 1","endPoint":{"x":60,"y":84,"heading":"constant","startDeg":180,"endDeg":150,"degrees":180},"controlPoints":[],"color":"#6A89D8"},{"name":"Path 2","endPoint":{"x":10,"y":60,"heading":"constant","reverse":false,"degrees":180},"controlPoints":[{"x":55,"y":58}],"color":"#78B5C8"},{"name":"Path 3","endPoint":{"x":60,"y":84,"heading":"constant","reverse":false,"degrees":180},"controlPoints":[{"x":55,"y":58}],"color":"#58B69B"},{"name":"Path 4","endPoint":{"x":11.5,"y":60,"heading":"linear","reverse":false,"startDeg":180,"endDeg":150,"degrees":180},"controlPoints":[{"x":52,"y":66}],"color":"#A7557D"},{"name":"Path 6","endPoint":{"x":60,"y":84,"heading":"constant","reverse":false,"degrees":180},"controlPoints":[],"color":"#ABBC6A"},{"name":"Path 6","endPoint":{"x":11.5,"y":60,"heading":"linear","reverse":false,"startDeg":180,"endDeg":150},"controlPoints":[{"x":52,"y":66}],"color":"#589887"},{"name":"Path 7","endPoint":{"x":60,"y":84,"heading":"constant","reverse":false,"degrees":180},"controlPoints":[],"color":"#A89B68"},{"name":"Path 8","endPoint":{"x":11.5,"y":60,"heading":"linear","reverse":false,"startDeg":180,"endDeg":150},"controlPoints":[{"x":52,"y":66}],"color":"#9C6BA7"},{"name":"Path 9","endPoint":{"x":60,"y":84,"heading":"constant","reverse":false,"degrees":180},"controlPoints":[],"color":"#96598A"},{"name":"Path 10","endPoint":{"x":17,"y":84,"heading":"constant","reverse":false,"degrees":180},"controlPoints":[],"color":"#9C6B79"},{"name":"Path 11","endPoint":{"x":54,"y":112,"heading":"tangential","reverse":false},"controlPoints":[],"color":"#8BBA9C"}]}
     */
    public static ColoredDecodePose START_POSE = new ColoredDecodePose(15.5, 114, Math.PI);
    public static ColoredDecodePose SHOOT_POSE = new ColoredDecodePose(60, 84, Math.PI);

    @Override
    public Pose startPose() {
        return null;
    }

    @Override
    public List<FollowParameters> paths(Follower follower) {
        return Collections.emptyList();
    }
}
