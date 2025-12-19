package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;

import java.util.Arrays;

public class PoseMirror implements FuturePose {
    private final Pose pose;
    private final AllianceColor color;
    private Pose blue;
    private Pose red;

    public PoseMirror(double posX, double posY, double heading, AllianceColor allianceColor) {
        this(new Pose(posX, posY, heading), allianceColor);
    }

    public PoseMirror(Pose pose, AllianceColor color) {
        if (color == AllianceColor.None) throw new RuntimeException("Uncolored ColoredDecodePose");
        this.pose = pose;
        this.color = color;
        if (color == AllianceColor.Red) red = pose;
        else if (color == AllianceColor.Blue) blue = pose;
    }

    public PoseMirror(double x, double y, double heading) {
        this(new Pose(x, y, heading), AllianceColor.Blue);
    }

    public PoseMirror(double x, double y) {
        this(new Pose(x, y), AllianceColor.Blue);
    }

    public Pose getPose(AllianceColor desiredColor) {
        if (desiredColor.equals(AllianceColor.Red)) {
            if (red == null) red = pose.mirror();
            return red;
        }

        if (blue == null) blue = pose.mirror();
        return blue;
    }

    @Override
    public Pose getPose() {
        return getPose(Globals.allianceColor);
    }

    public PoseMirror down(double inches) {
        return new PoseMirror(this.pose.plus(new Pose(0, -inches)), color);
    }

    public PoseMirror up(double inches) {
        return new PoseMirror(this.pose.plus(new Pose(0, inches)), color);
    }

    public PoseMirror towardsRedWall(double inches) {
        return new PoseMirror(this.pose.plus(new Pose(color == AllianceColor.Blue ? inches : -inches, 0)), color);
    }

    public PoseMirror towardsBlueWall(double inches) {
        return new PoseMirror(this.pose.plus(new Pose(color == AllianceColor.Red ? inches : -inches, 0)), color);
    }

    public static BezierCurve makeBezier(PoseMirror... poses) {
        return new BezierCurve(Arrays.stream(poses).map(PoseMirror::getPose).toArray(Pose[]::new));
    }

    public static BezierLine makeBezier(PoseMirror pose1, PoseMirror pose2) {
        return new BezierLine(pose1.getPose(), pose2.getPose());
    }

    public static BezierPoint makeBezier(PoseMirror pose) {
        return new BezierPoint(pose.getPose());
    }

    public static HeadingInterpolator mirror(HeadingInterpolator interpolation) {
        return t -> MathFunctions.normalizeAngle(Math.PI - interpolation.interpolate(t));
    }

    public AllianceColor getColor() {
        return color;
    }

    public Pose getUnmodifiedPose() {
        return pose;
    }

    public double getHeading() {
        return getPose().getHeading();
    }

    public PoseMirror offsetOppositeColor(Pose offset) {
        if (blue == null || red == null) getPose();

        if (color == AllianceColor.Red) blue = blue.plus(offset);
        if (color == AllianceColor.Blue) red = red.plus(offset);
        return this;
    }

    @Override
    public boolean initialized() {
        return true;
    }
}
