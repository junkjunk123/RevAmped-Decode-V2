package org.firstinspires.ftc.teamcode.utils.math;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;

public final class RobotKinematicsCalculator {
    /**
     * Gets the projected position after a given amount of time with a constant planar acceleration, assuming no angular acceleration
     * @param initialPose the robot's initial position
     * @param time the duration in seconds of the motion
     * @param initialVelocity the robot's initial twist
     * @param acceleration the robot's acceleration
     * @return the projected pose after the motion
     */
    public static Pose getProjectedPoseWithConstantLinearAcceleration(Pose initialPose, double time, Pose initialVelocity, Pose acceleration) {
        double omega = initialVelocity.getHeading();

        if (omega == 0) {
            double finalXPos = 0.5 * acceleration.getX() * time * time + initialVelocity.getX() * time + initialPose.getX();
            double finalYPos = 0.5 * acceleration.getY() * time * time + initialVelocity.getY() * time + initialPose.getY();
            return new Pose(finalXPos, finalYPos, initialPose.getHeading());
        }

        double theta_disp = omega * time;
        double theta_final = theta_disp + initialPose.getHeading();
        double theta_initial = initialPose.getHeading();

        double cos_initial = Math.cos(theta_initial);
        double sin_initial = Math.sin(theta_initial);
        double cos_disp = Math.cos(theta_disp);
        double sin_disp = Math.sin(theta_disp);

        Matrix initialRotation = new Matrix(new double[][]{
                {cos_initial, -sin_initial},
                {sin_initial, cos_initial}
        });

        double initialVelScale = 1 / omega;
        Matrix initialVelTransformation = new Matrix(new double[][]{
                {sin_disp, 1 - cos_disp},
                {cos_disp - 1, sin_disp}
        });
        Vector initialLinearVelVector = initialVelocity.getAsVector();
        initialLinearVelVector.transform(initialVelTransformation);
        initialLinearVelVector = initialLinearVelVector.times(initialVelScale);

        double accelScale = 1 / omega / omega;
        Matrix accelTransformation = new Matrix(new double[][]{
                {theta_disp * sin_disp + cos_disp - 1, -theta_disp * cos_disp + sin_disp},
                {theta_disp * cos_disp - sin_disp, theta_disp * sin_disp + 1 - cos_disp}
        });

        Vector accelerationVector = acceleration.getAsVector();
        accelerationVector.transform(accelTransformation);
        accelerationVector = accelerationVector.times(accelScale);

        Vector pose_disp = initialLinearVelVector.plus(accelerationVector);
        pose_disp.transform(initialRotation);
        Vector pose_final = pose_disp.plus(initialPose.getAsVector());
        return new Pose(theta_final, pose_final.getXComponent(), pose_final.getYComponent());
    }

    public static Pose getProjectedPoseWithConstantVelocity(
            Pose initialPose,
            double time,
            Vector velocity,
            double omega
    ) {
        double vx = velocity.getXComponent();
        double vy = velocity.getYComponent();

        double theta0 = initialPose.getHeading();

        // Pure translation (limit ω → 0)
        if (Math.abs(omega) < 1e-9) {
            double dx = (vx * Math.cos(theta0) - vy * Math.sin(theta0)) * time;
            double dy = (vx * Math.sin(theta0) + vy * Math.cos(theta0)) * time;

            return new Pose(
                    initialPose.getX() + dx,
                    initialPose.getY() + dy,
                    theta0
            );
        }

        double theta = omega * time;
        double sinT = Math.sin(theta);
        double cosT = Math.cos(theta);

        // V(theta) * v   (body → local frame)
        double dxLocal =
                ( sinT * vx - (1 - cosT) * vy ) / omega;
        double dyLocal =
                ( (1 - cosT) * vx + sinT * vy ) / omega;

        // Rotate into world frame
        double cos0 = Math.cos(theta0);
        double sin0 = Math.sin(theta0);

        double dxWorld = cos0 * dxLocal - sin0 * dyLocal;
        double dyWorld = sin0 * dxLocal + cos0 * dyLocal;

        return new Pose(
                initialPose.getX() + dxWorld,
                initialPose.getY() + dyWorld,
                theta0 + theta
        );
    }

}
