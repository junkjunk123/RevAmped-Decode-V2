package org.firstinspires.ftc.teamcode.math.projectile;

import static org.firstinspires.ftc.teamcode.math.calc.Angle.normalizeAnglePi;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.blueX;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.blueY;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.redX;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.redY;
import static org.firstinspires.ftc.teamcode.math.projectile.ShooterMath.velocityCompensation;
import static org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret.RAD_LIMIT;
import static org.firstinspires.ftc.teamcode.utils.Globals.allianceColor;
import static org.firstinspires.ftc.teamcode.utils.Globals.telemetry;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;

import java.util.Arrays;

import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

public class SimpleShooterMath {
    private final Localizer pinpoint;
    public static Pose APRIL_TAG_POSE_RED;
    public static Pose APRIL_TAG_POSE_BLUE;
    private double turretPos;
    private double hoodPos;
    private double flywheelVelocity;
    private final Interpolation2D velocityInterpolation;
    private final Interpolation2D hoodInterpolation;
    private final Interpolation2D airTime;
    private final Interpolation2D turretInterpolation;
    private static final double[] DIST_Y = {15.0, 39.0, 63.0};
    private static final double[] DIST_X = {24.0, 60.0, 96.0};
    public static double HOOD_0_DEG = 31.5;
    public static double HOOD_POS_TO_DEG_SLOPE = 20.26578947368421;
    public static final int SOTM_ITERATIONS = 10;

    public SimpleShooterMath(Localizer pinpoint) {
        this.pinpoint = pinpoint;
        APRIL_TAG_POSE_BLUE = new Pose(blueX, blueY);
        APRIL_TAG_POSE_RED = new Pose(redX, redY);

        double[][] hoodPos = {
                {0.02, 0.08, 0.20},
                {0.02, 0.18, 0.20},
                {0.12, 0.27, 0.29}
        };
        hoodPos = new Matrix(hoodPos).transposed().getMatrix();

        double[][] flywheelVel = {
                { 600, 665, 825 },
                { 635, 740, 850 },
                { 710, 815, 875}
        };
        flywheelVel = new Matrix(flywheelVel).transposed().getMatrix();

        double[][] airTimes = {
                {0, 0, 0},
                {0, 0, 0},
                {0, 0, 0}
        };
        airTimes = new Matrix(airTimes).transposed().getMatrix();

        //(0,0) -> (3, 0) : x increase
        //(0, 0) -> (0, 3) down : y decrease
        double[][] turretPos = {
                {-0.55, -0.23, -0.15},
                {-1.0, -0.6, -0.45},
                {-1.15, -0.77, -0.58}
        };
        turretPos = new Matrix(turretPos).transposed().getMatrix();

        double[] X_BLUE = new double[] {-DIST_X[0], -DIST_X[1], -DIST_X[2]};

        double[][] hoodSine = Arrays.stream(hoodPos)
                .map(row -> Arrays.stream(row)
                        .map(this::hoodPosToSin)
                        .toArray()
                )
                .toArray(double[][]::new);

        velocityInterpolation = new BilinearInterpolation(DIST_X, DIST_Y, flywheelVel);
        hoodInterpolation = new BilinearInterpolation(DIST_X, DIST_Y, hoodSine);
        airTime = new BilinearInterpolation(DIST_X, DIST_Y, airTimes);
        turretInterpolation = new BilinearInterpolation(Globals.allianceColor == AllianceColor.Red ? DIST_X : X_BLUE, DIST_Y, turretPos);
    }

    public void update(boolean trackTurret, boolean trackHood) {
        APRIL_TAG_POSE_BLUE = new Pose(blueX, blueY);
        APRIL_TAG_POSE_RED = new Pose(redX, redY);

        if (trackHood || trackTurret) {
            Pose targetPos = allianceColor == AllianceColor.Red ? APRIL_TAG_POSE_RED : APRIL_TAG_POSE_BLUE;
            Pose currentPos = pinpoint.getPose();
            Vector displacement = getDispVector(targetPos, currentPos);

            if (trackTurret) {
                if (!velocityCompensation) {
                    turretPos = getTurretPos(displacement);
                } else {
                    Pose currentVelocity = pinpoint.getVelocity();
                    turretPos = getTurretPos(getDispVector(targetPos, iteratePose(currentPos, currentVelocity)));
                }
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

    private Pose iteratePose(Pose currentPos, Pose fieldVelocity) {
        Pose virtualPose = currentPos;

        for (int i = 0; i < SOTM_ITERATIONS; i++) {
            double airTime = this.airTime.interpolate(virtualPose.getX(), virtualPose.getY());
            virtualPose = currentPos.minus(fieldVelocity.scale(airTime));
        }

        return virtualPose;
    }

    public double getTurretPos() {
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

    public double getTurretPos(Vector offset, double heading) {
        double pos = turretInterpolation.interpolate(offset.getXComponent(), offset.getYComponent());
        double delta = normalizeAnglePi(pos - heading);
        double ticks = ServoTurret.radToTicks(delta);
        return Range.clip(ticks, Math.min(ServoTurret.LEFT_TICKS_LIMIT, ServoTurret.RIGHT_TICKS_LIMIT),
                Math.max(ServoTurret.RIGHT_TICKS_LIMIT, ServoTurret.LEFT_TICKS_LIMIT));
    }

    public double getTurretPos(Vector offset) {
        return getTurretPos(offset, pinpoint.getPose().getHeading());
    }
}
