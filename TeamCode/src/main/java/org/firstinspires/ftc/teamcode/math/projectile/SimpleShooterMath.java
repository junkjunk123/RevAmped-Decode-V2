package org.firstinspires.ftc.teamcode.math.projectile;
import static org.firstinspires.ftc.teamcode.math.calc.Angle.normalizeAnglePi;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.BALL_LAUNCH_MS;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.blueNegativeX;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.blueNegativeY;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.redPositiveX;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.redPositiveY;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.velocityCompensation;
import static org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret.RAD_LIMIT;
import static org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret.TICKS_LIMIT;
import static org.firstinspires.ftc.teamcode.utils.Globals.allianceColor;
import static org.firstinspires.ftc.teamcode.utils.Globals.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.math.MathUtil;
import org.firstinspires.ftc.teamcode.math.RobotKinematicsCalculator;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;

import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

public class SimpleShooterMath {
    private final Follower follower;
    public static Pose APRIL_TAG_POSE_RED;
    public static Pose APRIL_TAG_POSE_BLUE;
    private int turretPos;
    private double hoodPos;
    private double flywheelVelocity;
    private final Interpolation2D velocityInterpolation;
    private final Interpolation2D hoodInterpolation;
    private final Interpolation2D airTime;
    private static final double[] DIST_X = {24.0, 60.0};
    private static final double[] DIST_Y = {15.0, 39.0, 63.0};
    public static double HOOD_0_DEG = 28.9;
    public static double HOOD_POS_TO_DEG_SLOPE = 20.26578947368421;

    public SimpleShooterMath(Follower follower) {
        this.follower = follower;
        APRIL_TAG_POSE_BLUE = new Pose(blueNegativeX, blueNegativeY);
        APRIL_TAG_POSE_RED = new Pose(redPositiveX, redPositiveY);

        double[][] hoodSine = {
                { 0.4925446801631296, 0.5199946813957746, 0.6379734257885634 },
                { 0.6019030541429722, 0.6297664634859653, 0.6434048835870022 }
        };

        double[][] flywheelVel = {
                { 700.0, 725.0, 925.0 },
                { 850.0, 875.0, 950.0 }
        };

        double[][] airTime = {
                {0, 0, 0},
                {0, 0, 0}
        };

        velocityInterpolation = new BilinearInterpolation(DIST_X, DIST_Y, flywheelVel);
        hoodInterpolation = new BilinearInterpolation(DIST_X, DIST_Y, hoodSine);
        this.airTime = new BilinearInterpolation(DIST_X, DIST_Y, airTime);
    }

    public void update(boolean trackTurret, boolean trackHood) {
        if (trackHood || trackTurret) {
            Pose targetPos = allianceColor == AllianceColor.Red ? APRIL_TAG_POSE_RED : APRIL_TAG_POSE_BLUE;
            Pose currentPos = follower.getPose();
            Vector displacement = getDispVector(targetPos, currentPos);

            if (trackTurret) {
                double deltaAngle;
                if (!velocityCompensation) {
                    deltaAngle = angleTurretTo(displacement);
                } else {
                    Pose currentVelocity = follower.
                    deltaAngle = angleTurretTo(iterateOffset(targetPos, currentPos));
                }
                turretPos = (int) Range.clip(deltaAngle * TICKS_LIMIT / RAD_LIMIT, -TICKS_LIMIT, TICKS_LIMIT);
            }

            if (trackHood) {
                double xDist = Math.abs(displacement.getXComponent());
                double yDist = Math.abs(displacement.getYComponent());
                flywheelVelocity = velocityInterpolation.interpolate(xDist, yDist);
                flywheelVelocity = Range.clip(flywheelVelocity,0, Flywheel.MAX_VELOCITY);
                double hoodSine = hoodInterpolation.interpolate(xDist, yDist);
                hoodSine = Range.clip(hoodSine, 0, 1);
                double hoodDeg = Math.toDegrees(Math.asin(hoodSine));
                hoodPos = (hoodDeg - HOOD_0_DEG) / HOOD_POS_TO_DEG_SLOPE;
                hoodPos = Range.clip(hoodPos, 0, 1);
            }
        }
    }

    public void reset(int turretPos, float hoodPos) {
        this.turretPos = turretPos;
        this.hoodPos = hoodPos;
    }

    private double angleTurretTo(Vector offset) {
        double targetAngle = offset.getTheta() + Math.PI;
        double currentAngle = follower.getPose().getHeading();
        double deltaAngle = normalizeAnglePi(targetAngle - currentAngle);
        return Range.clip(deltaAngle, -RAD_LIMIT, RAD_LIMIT);
    }

    private double angleTurretTo(Pose offset) {
        double targetAngle = offset.getAsVector().getTheta() + Math.PI;
        double currentAngle = offset.getHeading();
        double deltaAngle = normalizeAnglePi(targetAngle - currentAngle);
        return Range.clip(deltaAngle, -RAD_LIMIT, RAD_LIMIT);
    }

    private Vector getDispVector(Pose target, Pose current) {
        return target.minus(current).getAsVector();
    }

    private Pose iterateOffset(Pose targetPos, Pose currentPos, Pose fieldVelocity) {
        Matrix inverseRotation = MathUtil.rotMatrix(currentPos.getHeading()).transposed();
        Vector velVector = fieldVelocity.getAsVector();
        Vector robotLinearVel = velVector.transform(inverseRotation);
        Pose robotVelPose = new Pose(robotLinearVel.getXComponent(), robotLinearVel.getYComponent(), fieldVelocity.getHeading());

        Pose currentIteration = targetPos.minus(currentPos);
        for (int i = 0; i < 10; i++) {
            Pose disp = currentIteration.minus(currentPos);
            double shotTime = airTime.interpolate(disp.getX(), disp.getY());
            currentIteration = targetPos.minus(RobotKinematicsCalculator.getProjectedPoseWithConstantVelocity(
                    currentPos,
                    shotTime,
                    robotVelPose
            ));
        }

        return currentIteration;
    }

    public int getTurretPos() {
        return turretPos;
    }

    public double getHoodPos() {
        return hoodPos;
    }

    public double getFlywheelVelocity() {
        return flywheelVelocity;
    }
}
