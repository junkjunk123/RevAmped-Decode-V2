package org.firstinspires.ftc.teamcode.math.projectile;
import static org.firstinspires.ftc.teamcode.math.calc.Angle.normalizeAnglePi;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.BALL_LAUNCH_MS;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.blueX;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.blueY;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.redX;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.redY;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.velocityCompensation;
import static org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret.RAD_LIMIT;
import static org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret.TICKS_LIMIT;
import static org.firstinspires.ftc.teamcode.utils.Globals.allianceColor;
import static org.firstinspires.ftc.teamcode.utils.Globals.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.math.MathUtil;
import org.firstinspires.ftc.teamcode.math.RobotKinematicsCalculator;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

import java.util.Arrays;

import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

public class SimpleShooterMath {
    private final Localizer pinpoint;
    public static Pose APRIL_TAG_POSE_RED;
    public static Pose APRIL_TAG_POSE_BLUE;
    private int turretPos;
    private double hoodPos;
    private double flywheelVelocity;
    private final Interpolation2D velocityInterpolation;
    private final Interpolation2D hoodInterpolation;
    private final Interpolation2D airTime;
    private static final double[] DIST_X = {24.0, 60.0, 96.0};
    private static final double[] DIST_Y = {15.0, 39.0, 63.0};
    public static double HOOD_0_DEG = 31.5;
    public static double HOOD_POS_TO_DEG_SLOPE = 20.26578947368421;

    public SimpleShooterMath(Localizer pinpoint) {
        this.pinpoint = pinpoint;
        APRIL_TAG_POSE_BLUE = new Pose(blueX, blueY);
        APRIL_TAG_POSE_RED = new Pose(redX, redY);

        double[][] hoodPos = {
                {0.02, 0.3, 0.45},
                {0.2, 0.35, 0.5},
                {0.35, 0.38, 0.53}
        };

        double[][] flywheelVel = {
                { 590, 700, 865 },
                { 640, 720, 920 },
                { 720, 780, 925}
        };

        double[][] airTime = {
                {0, 0, 0},
                {0, 0, 0},
                {0, 0, 0}
        };

        double[][] hoodSine = Arrays.stream(hoodPos)
                .map(row -> Arrays.stream(row)
                        .map(this::hoodPosToSin)
                        .toArray()
                )
                .toArray(double[][]::new);

        velocityInterpolation = new BilinearInterpolation(DIST_Y, DIST_X, flywheelVel);
        hoodInterpolation = new BilinearInterpolation(DIST_Y, DIST_X, hoodSine);
        this.airTime = new BilinearInterpolation(DIST_Y, DIST_X, airTime);
    }

    public void update(boolean trackTurret, boolean trackHood) {
        if (trackHood || trackTurret) {
            Pose targetPos = allianceColor == AllianceColor.Red ? APRIL_TAG_POSE_RED : APRIL_TAG_POSE_BLUE;
            Pose currentPos = pinpoint.getPose();
            Vector displacement = getDispVector(targetPos, currentPos);

            if (trackTurret) {
                double deltaAngle;
                if (!velocityCompensation) {
                    deltaAngle = angleTurretTo(displacement);
                } else {
                    Pose currentVelocity = pinpoint.getVelocity();
                    deltaAngle = angleTurretTo(iterateOffset(targetPos, currentPos, currentVelocity));
                }
                turretPos = (int) Range.clip(deltaAngle * TICKS_LIMIT / RAD_LIMIT, -TICKS_LIMIT, TICKS_LIMIT);
            }

            if (trackHood) {
                double xDist = Math.abs(displacement.getXComponent());
                double yDist = Math.abs(displacement.getYComponent());
                flywheelVelocity = velocityInterpolation.interpolate(xDist-10, yDist-10);
                flywheelVelocity = Range.clip(flywheelVelocity,0, Flywheel.MAX_VELOCITY);
                double hoodSine = hoodInterpolation.interpolate(xDist-10, yDist-10);
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
        double currentAngle = pinpoint.getPose().getHeading();
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
            double shotTime = airTime.interpolate(Math.abs(currentIteration.getX()), Math.abs(currentIteration.getY()));
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

    private double hoodPosToSin(double pos) {
        double hoodDeg = pos * HOOD_POS_TO_DEG_SLOPE + HOOD_0_DEG;
        return Math.sin(Math.toRadians(hoodDeg));
    }
}
