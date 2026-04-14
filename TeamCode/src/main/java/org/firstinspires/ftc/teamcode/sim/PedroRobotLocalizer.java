package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.utils.math.RobotKinematicsCalculator;

public class PedroRobotLocalizer implements Localizer {
    private Pose currentPose;
    private Vector velocity;
    private double angularVelocity;

    @Override
    public Pose getPose() {
        return currentPose;
    }

    @Override
    public Pose getVelocity() {
        return new Pose(velocity.getXComponent(), velocity.getYComponent(), angularVelocity);
    }

    @Override
    public Vector getVelocityVector() {
        return getVelocity().getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        setPose(setStart);
    }

    @Override
    public void setPose(Pose setPose) {
        currentPose = setPose;
    }

    @Override
    public void update() {

    }

    public void update(Vector twist, double angularVel) {
        currentPose = RobotKinematicsCalculator.getProjectedPoseWithConstantVelocity(currentPose, PedroSimulator.dt, twist, angularVel);
        velocity = twist;
    }

    @Override
    public double getTotalHeading() {
        return 0;
    }

    @Override
    public double getForwardMultiplier() {
        return 0;
    }

    @Override
    public double getLateralMultiplier() {
        return 0;
    }

    @Override
    public double getTurningMultiplier() {
        return 0;
    }

    @Override
    public void resetIMU() throws InterruptedException {

    }

    @Override
    public double getIMUHeading() {
        return 0;
    }

    @Override
    public boolean isNAN() {
        return false;
    }
}
