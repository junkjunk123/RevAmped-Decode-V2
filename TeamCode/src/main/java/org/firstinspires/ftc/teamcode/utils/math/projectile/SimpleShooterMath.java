package org.firstinspires.ftc.teamcode.utils.math.projectile;

import static org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread.velocityCompensation;
import static org.firstinspires.ftc.teamcode.utils.Globals.allianceColor;
import static org.firstinspires.ftc.teamcode.utils.Globals.telemetry;
import static org.firstinspires.ftc.teamcode.utils.math.calc.Angle.normalizeAnglePi;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.GyroThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretMTI;
import org.firstinspires.ftc.teamcode.pedro.octoquad.OctoQuadLocalizer;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.math.MathUtil;

import java.util.Arrays;

import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

@Config
public class SimpleShooterMath {
    private final OctoQuadLocalizer localizer;
    public static Pose APRIL_TAG_POSE_RED;
    public static Pose APRIL_TAG_POSE_BLUE;
    private double turretPos;
    private double hoodPos;
    public static double hoodOffset = 0;
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
    public static double CALIBRATION_ANGLE = 0;
    public static double ANGULAR_CONSTANT = 0.05;

    public static double blueX = 9.5;
    public static double blueY = 135;
    public static double redX = 120.5;
    public static double redY = 120;

    public SimpleShooterMath(Localizer localizer) {
        this.localizer = (OctoQuadLocalizer) localizer;
        APRIL_TAG_POSE_BLUE = new Pose(blueX, blueY);
        APRIL_TAG_POSE_RED = new Pose(redX, redY);

        //(0,0) -> (3, 0) : x increase
        //(0, 0) -> (0, 3) down : y decrease

        double[][] hoodPos = {
                //Left-Top is closet to target goal
                {0.020, 0.235, 0.373},
                {0.118, 0.294, 0.333},
                {0.314, 0.314, 0.373}
        };
        hoodPos = new Matrix(hoodPos).transposed().getMatrix();

        double[][] flywheelVel = {
                {700, 900, 1050},
                {800, 940, 1050},
                {900, 1000, 1050}
        };
        flywheelVel = new Matrix(flywheelVel).transposed().getMatrix();

        double[][] airTimes = {
                {0.25, 0.44, 0.76},
                {0.44, 0.45, 0.87},
                {0.44, 0.66, 0.90}
        };
        airTimes = new Matrix(airTimes).transposed().getMatrix();

        double[][] turretPos = {
                {0.598, 0.550, 0.524},
                {0.670, 0.598, 0.565},
                {0.704, 0.639, 0.598}
        };

        turretPos = new Matrix(Arrays.stream(turretPos)
                .map(d -> Arrays.stream(d).map(ServoTurretMTI::ticksToRad).toArray())
                .toArray(double[][]::new))
                .transposed().getMatrix();

        double[][] hoodSine = Arrays.stream(hoodPos)
                .map(row -> Arrays.stream(row)
                        .map(this::hoodPosToSin)
                        .toArray()
                )
                .toArray(double[][]::new);

        velocityInterpolation = new BilinearInterpolation(DIST_X, DIST_Y, flywheelVel);
        hoodInterpolation = new BilinearInterpolation(DIST_X, DIST_Y, hoodSine);
        airTime = new BilinearInterpolation(DIST_X, DIST_Y, airTimes);
        turretInterpolation = new BilinearInterpolation(DIST_X, DIST_Y, turretPos);
    }

    public void update(boolean trackTurret, boolean trackHood) {
        APRIL_TAG_POSE_BLUE = new Pose(blueX, blueY);
        APRIL_TAG_POSE_RED = new Pose(redX, redY);

        if (trackHood || trackTurret) {
            Pose targetPos = allianceColor == AllianceColor.Red ? APRIL_TAG_POSE_RED : APRIL_TAG_POSE_BLUE;
            Pose currentPos = localizer.getPose();
//            telemetry.addData("math pose",currentPos);
            Vector displacement = getDispVector(targetPos, currentPos);
//            telemetry.addData("currentPos", currentPos);
//            telemetry.addData("disp vector", displacement);
            if (currentPos.getY() < Robot.FAR_SHOOT_THRESHOLD_Y && !Robot.shootingFar){
                Robot.shootingFar = true;
            }
            if (currentPos.getY() >= Robot.FAR_SHOOT_THRESHOLD_Y && Robot.shootingFar){
                Robot.shootingFar = false;
            }
            if (trackTurret) {
                if (!velocityCompensation) {
                    turretPos = getTurretPos(displacement);
                } else {
                    Pose currentVelocity = localizer.getVelocity();
                    turretPos = getTurretPos(getDispVector(targetPos, iteratePose(currentPos, currentVelocity)));
                }
                double omegaComp = localizer.getAngularVelocity() * ANGULAR_CONSTANT;
                turretPos = ServoTurretMTI.radToTicks(MathUtil.normalizeAnglePi(ServoTurretMTI.ticksToRad(turretPos) - omegaComp));
            }

            if (trackHood) {
                double xDist = Math.abs(displacement.getXComponent());
                double yDist = Math.abs(displacement.getYComponent());
                flywheelVelocity = velocityInterpolation.interpolate(xDist, yDist);
                //goon
                if (!Globals.isTeleOp) flywheelVelocity+=100;
                flywheelVelocity = Range.clip(flywheelVelocity,0, Flywheel.MAX_VELOCITY);
                double hoodSine = hoodInterpolation.interpolate(xDist, yDist);
                hoodSine = Range.clip(hoodSine, 0, 1);
                double hoodDeg = Math.toDegrees(Math.asin(hoodSine));
                hoodPos = (hoodDeg - HOOD_0_DEG) / HOOD_POS_TO_DEG_SLOPE;
                hoodPos += hoodOffset;
                hoodPos = Range.clip(hoodPos, 0, 1);
            }
        }
    }

    public void reset(int turretPos, float hoodPos) {
        this.turretPos = turretPos;
        this.hoodPos = hoodPos;
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
        //double pos = turretInterpolation.interpolate(Globals.allianceColor.equals(AllianceColor.Red) ? offset.getXComponent() :
                //-offset.getXComponent(), offset.getYComponent());
        double pos = offset.getTheta();
        double delta = normalizeAnglePi(pos - heading + CALIBRATION_ANGLE);
        double ticks = ServoTurretMTI.radToTicks(delta);
        ticks -= GyroThread.NEUTRAL_OFFSET * Math.signum(ServoTurretMTI.REST - pos);
        return Range.clip(ticks, Math.min(ServoTurretMTI.LEFT_TICKS_LIMIT, ServoTurretMTI.RIGHT_TICKS_LIMIT),
                Math.max(ServoTurretMTI.RIGHT_TICKS_LIMIT, ServoTurretMTI.LEFT_TICKS_LIMIT));
    }

    public double getTurretPos(Vector offset) {
        return getTurretPos(offset, localizer.getPose().getHeading());
    }
}
