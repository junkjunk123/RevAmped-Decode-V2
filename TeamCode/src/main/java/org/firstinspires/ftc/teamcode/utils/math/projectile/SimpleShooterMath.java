package org.firstinspires.ftc.teamcode.utils.math.projectile;

import static org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread.TURRET_OFFSET;
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
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretMTI;
import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
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
    public static double hoodCompOffset = 0;
    public static double SOTMOffset = 0;
    public static double turretCompOffset = 0;
    private double flywheelVelocity;
    private final Interpolation2D velocityInterpolation;
    private final Interpolation2D hoodInterpolation;
    private final Interpolation2D airTime;
    private final Interpolation2D turretInterpolation;
    private static final double[] DIST_Y = {24.0, 48.0, 72.0};
    private static final double[] DIST_X = {36, 72, 108};
    public static double HOOD_0_DEG = 31.5;
    public static double HOOD_POS_TO_DEG_SLOPE = 20.26578947368421;
    public static final int SOTM_ITERATIONS = 10;
    public static double CALIBRATION_ANGLE = 0;
    public static double ANGULAR_CONSTANT = 0.05;
    public static double K_time = 1.0;
    public static double K_V = 0.1;
    public static double blueX = 0;
    public static double blueY = 144;
    public static double redX = 144;
    public static double redY = 144;
    public static double DT = 0.001;
    public static double launchToVel; //launch vel to flywheel vel
    public static double velOffset;
    public static double ticksPerRad; //hood angle
    public static double shootingAngleRelativeToHood = Math.PI/2;
    private double maxFlywheelVel;
    public static ColoredDecodePose offsetPose = new ColoredDecodePose(-6, -6);
    public static double K_flywheelVel;
    private HoodInverseKinematics inverseKinematics;

    public SimpleShooterMath(Localizer localizer) {
        inverseKinematics = new HoodInverseKinematics();
        this.localizer = (OctoQuadLocalizer) localizer;
        APRIL_TAG_POSE_BLUE = new Pose(blueX, blueY);
        APRIL_TAG_POSE_RED = new Pose(redX, redY);

        //(0,0) -> (3, 0) : x increase
        //(0, 0) -> (0, 3) down : y decrease

        double[][] hoodPos = {
                //Left-Top is closet to target goal
                //offset by +0.094
                {0.114, 0.329, 0.467},
                {0.172, 0.388, 0.427},
                {0.408, 0.408, 0.467}
        };
        hoodPos = new Matrix(hoodPos).transposed().getMatrix();

        double[][] flywheelVel = {
                {670, 870, 1035},
                {760, 940, 1035},
                {870, 1020, 1050}
        };

//        double[][] flywheelVel = { //in inch/s of ball launch velocity (NOT FLYWHEEL VELOCITY)
//                {194, 225, 254},
//                {210, 230, 257},
//                {230, 253, 265}
//        };
        flywheelVel = new Matrix(flywheelVel).transposed().getMatrix();

        for (double[] doubles : flywheelVel)
            for (double d : doubles)
                maxFlywheelVel = Math.max(maxFlywheelVel, d);

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

    public void update(boolean trackTurret, boolean trackHood, Flywheel flywheel) {
        APRIL_TAG_POSE_BLUE = new Pose(blueX, blueY);
        APRIL_TAG_POSE_RED = new Pose(redX, redY);

        if (trackHood || trackTurret) {
            Pose targetPos = allianceColor == AllianceColor.Red ? APRIL_TAG_POSE_RED : APRIL_TAG_POSE_BLUE;
            Pose currentPos = localizer.getPose();
            Pose currentVelocity = localizer.getVelocity();
//            telemetry.addData("math pose",currentPos);
            Vector displacement = getDispVector(targetPos, currentPos);
            Vector linearVel = currentVelocity.getAsVector();
            double velMag = linearVel.getMagnitude();

            if (velocityCompensation) {
                if (velMag > Hood.HOOD_COMP_SOTM_THRESHOLD && SimpleShooterMath.SOTMOffset == 0) {
                    if (linearVel.dot(displacement) > 0)
                        SimpleShooterMath.SOTMOffset = Hood.HOOD_COMP_SOTM;
                    else
                        SimpleShooterMath.SOTMOffset = Hood.HOOD_COMP_SOTM + Hood.HOOD_COMP_SOTM_BACKWARDS;
                }
                if (velMag <= Hood.HOOD_COMP_SOTM_THRESHOLD && SimpleShooterMath.SOTMOffset != 0) {
                    SimpleShooterMath.SOTMOffset = 0;
                }
            }

//            telemetry.addData("currentPos", currentPos);
//            telemetry.addData("disp vector", displacement);
            if (currentPos.getY() < Robot.FAR_SHOOT_THRESHOLD_Y && !Robot.shootingFar){
                Robot.shootingFar = true;
            }

            if (currentPos.getY() >= Robot.FAR_SHOOT_THRESHOLD_Y && Robot.shootingFar){
                Robot.shootingFar = false;
            }

            if (trackTurret) {
                turretPos = computeTurretPos(displacement, currentPos, currentVelocity, targetPos);
                double nextTurretPos = computeNextTurretPos(currentPos, currentVelocity, targetPos);
                double turretPosDeriv = (nextTurretPos - turretPos) / DT;
                double omegaComp = localizer.getAngularVelocity() * ANGULAR_CONSTANT;
                double velFeedforward = turretPosDeriv * K_V;
                turretPos = ServoTurretMTI.radToTicks(MathUtil.normalizeAnglePi(ServoTurretMTI.ticksToRad(turretPos + TURRET_OFFSET) - omegaComp) + velFeedforward);
                turretPos += turretCompOffset;
            }

            if (trackHood) {
                double xDist = Math.abs(displacement.getXComponent());
                double yDist = Math.abs(displacement.getYComponent());
                flywheelVelocity = velocityInterpolation.interpolate(xDist, yDist);
//                flywheelVelocity = launch*launchToVel;
                flywheelVelocity = Range.clip(flywheelVelocity,0, Math.min(maxFlywheelVel, Flywheel.MAX_VELOCITY));
                flywheelVelocity += Robot.flywheelFineTune;
//
//                inverseKinematics.setDistance(displacement.getMagnitude());
//                inverseKinematics.setFlywheelVelocity(flywheel.getVelocity() * K_flywheelVel);
//                inverseKinematics.calculateHoodAngle();
//                hoodPos = (inverseKinematics.getAngle()* ticksPerRad)/255;
//                hoodPos = Range.clip(hoodPos, (double) 5/255, (double) 168/255);
                
                double hoodSine = hoodInterpolation.interpolate(xDist, yDist);
                hoodSine = Range.clip(hoodSine, 0, 1);
                double hoodDeg = Math.toDegrees(Math.asin(hoodSine));
                hoodPos = (hoodDeg - HOOD_0_DEG) / HOOD_POS_TO_DEG_SLOPE;
                hoodPos += hoodCompOffset+SOTMOffset+Robot.hoodFineTune;
                hoodPos = Range.clip(hoodPos, 0, 1);
            }
        }
    }

    private double computeTurretPos(Vector displacement, Pose currentPos, Pose currentVelocity, Pose targetPos) {
        if (!velocityCompensation)
            return getTurretPos(displacement);

        return getTurretPos(getDispVector(targetPos, iteratePose(currentPos, currentVelocity, targetPos)));
    }

    private double computeNextTurretPos(Pose currentPos, Pose currentVelocity, Pose targetPos) {
        Pose nextPos = currentPos.plus(currentVelocity.scale(DT));
        Vector displacement = targetPos.minus(nextPos).getAsVector();
        return computeTurretPos(displacement, nextPos, currentVelocity, targetPos);
    }

    public void reset(int turretPos, float hoodPos) {
        this.turretPos = turretPos;
        this.hoodPos = hoodPos;
    }

    private Vector getDispVector(Pose target, Pose current) {
        return target.minus(current).getAsVector();
    }

    private Pose iteratePose(Pose currentPos, Pose fieldVelocity, Pose targetPos) {
        if (fieldVelocity.getX() * fieldVelocity.getX() + fieldVelocity.getY() * fieldVelocity.getY() < 1) return currentPos;
        Pose virtualPose = currentPos;

        for (int i = 0; i < SOTM_ITERATIONS; i++) {
            double airTime = airTime(currentPos, targetPos);
            virtualPose = currentPos.plus(fieldVelocity.scale(airTime));
        }

        return virtualPose;
    }

    private double airTime(Pose current, Pose target) {
        Vector disp = current.minus(target).getAsVector();
        return K_time * airTime.interpolate(Math.abs(disp.getXComponent()), Math.abs(disp.getYComponent()));
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
