package org.firstinspires.ftc.teamcode.revamped.math.projectile;
import static org.firstinspires.ftc.teamcode.revamped.math.calc.Angle.normalizeAnglePi;
import static org.firstinspires.ftc.teamcode.revamped.math.calc.Angle.servoPosFromRad;
import static org.firstinspires.ftc.teamcode.revamped.utils.Globals.allianceColor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.revamped.math.RobotKinematicsCalculator;
import org.firstinspires.ftc.teamcode.revamped.math.calc.Angle;
import org.firstinspires.ftc.teamcode.revamped.math.calc.PoseDifferentiator;
import org.firstinspires.ftc.teamcode.revamped.math.calc.Vector3D;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.HoodServo;
import org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter.TurretMotor;
import org.firstinspires.ftc.teamcode.revamped.utils.AllianceColor;

public class ShooterMath {
    private final Follower follower;
    public static Pose APRIL_TAG_POSE_RED;
    public static Pose APRIL_TAG_POSE_BLUE_POSITIVE;
    public static Pose APRIL_TAG_POSE_RED_NEGATIVE;
    public static Pose APRIL_TAG_POSE_BLUE;
    public static double LAUNCH_ZONE_HEIGHT = 58;

    public static double bluePositiveX = 14; //added 8 inch
    public static double bluePositiveY = 145; //subtracted 4 inch
    public static double blueNegativeX = 15; //added 18 inch
    public static double blueNegativeY = 133;
    public static double redPositiveX = 119;
    public static double redPositiveY = 133;
    public static double redNegativeX = 151;
    public static double redNegativeY = 180;

    public static double BALL_LAUNCH_MS = 400;
    public static double LAUNCH_ETA = 0.9;

    public static boolean velocityCompensation;

    private PoseDifferentiator accelerationCalculator;

    public double turretPos;
    public double hoodPos;
    public double confidence;

    public ShooterMath(Follower follower) {
        this.follower = follower;
        accelerationCalculator = new PoseDifferentiator(Pose::new, follower.poseTracker.getLocalizer()::getVelocity);
        APRIL_TAG_POSE_BLUE = new Pose(blueNegativeX, blueNegativeY);
        APRIL_TAG_POSE_BLUE_POSITIVE = new Pose(bluePositiveX, bluePositiveY);
        APRIL_TAG_POSE_RED = new Pose(redPositiveX, redPositiveY);
        APRIL_TAG_POSE_RED_NEGATIVE = new Pose(redNegativeX, redNegativeY);
    }

    public void update(boolean trackTurret, boolean trackHood, double flywheelVelocity) {
        Pose targetPos = allianceColor == AllianceColor.Red ? APRIL_TAG_POSE_RED : APRIL_TAG_POSE_BLUE;
        Pose currentPos = follower.getPose();
        Pose robotVelPose = follower.poseTracker.getLocalizer().getVelocity();
        Vector robotVelocity = robotVelPose.getAsVector();
        Pose robotAcceleration = accelerationCalculator.calculate(robotVelPose);
        Pose projectedRobotPose = RobotKinematicsCalculator.getProjectedPoseWithConstantLinearAcceleration(
                currentPos,
                BALL_LAUNCH_MS / 1000.0,
                robotVelPose,
                robotAcceleration
        );
        Vector projectedRobotVelocity = robotVelocity.plus(robotAcceleration.getAsVector().times(BALL_LAUNCH_MS / 1000));

        double deltaAngle;

        if (trackTurret) {
            if (!velocityCompensation || robotVelocity.getMagnitude() < 8) {
                deltaAngle = angleTurretTo(currentPos, targetPos);

                if ((deltaAngle > 0 && allianceColor == AllianceColor.Blue) || (deltaAngle < 0 && allianceColor == AllianceColor.Red)) {
                    Pose targetCorrectedPose = allianceColor == AllianceColor.Red ? APRIL_TAG_POSE_RED_NEGATIVE : APRIL_TAG_POSE_BLUE_POSITIVE;
                    deltaAngle = angleTurretTo(currentPos, targetCorrectedPose);
                }

                turretPos = deltaAngle;
            } else if (velocityCompensation) {
                Vector offset = targetPos.minus(projectedRobotPose).getAsVector();
                Vector iHat = offset.normalize();
                Vector jHat = new Vector();
                jHat.setOrthogonalComponents(-iHat.getYComponent(), iHat.getXComponent());
                Vector invertediHat = new Vector();
                invertediHat.setOrthogonalComponents(iHat.getYComponent(), iHat.getXComponent());
                double normalizedTurretNormalComponent = Range.clip(projectedRobotVelocity.dot(invertediHat) / flywheelVelocity, -1, 1);
                Vector flywheelDirectionVector = new Vector();
                flywheelDirectionVector.setOrthogonalComponents(
                        Math.sqrt(1 - Math.pow(normalizedTurretNormalComponent, 2)),
                        normalizedTurretNormalComponent
                );
                Matrix inverseRotMatrix = new Matrix(new double[][] {
                        {iHat.getXComponent(), iHat.getYComponent()},
                        {jHat.getXComponent(), jHat.getYComponent()}
                });
                double targetAngle = flywheelDirectionVector.transform(inverseRotMatrix).getTheta() + Math.PI;
                deltaAngle = normalizeAnglePi(targetAngle - projectedRobotPose.getHeading());
                deltaAngle = Range.clip(deltaAngle, -TurretMotor.RAD_LIMIT, TurretMotor.RAD_LIMIT);
                turretPos = deltaAngle;
            }
        }

        if (trackHood) {
            Vector3D target = Vector3D.get3DPosition(targetPos, LAUNCH_ZONE_HEIGHT);
            Pose robotPose = velocityCompensation? projectedRobotPose : currentPos;
            Vector robotVel = velocityCompensation? projectedRobotVelocity : robotVelocity;
            Pose targetDisplacements = new Pose (
                    robotPose.distanceFrom(new Pose(target.getX(), target.getY())),
                    target.getZ() - LAUNCH_ZONE_HEIGHT
            );
            double initialVelocity = calcInitialLaunchVelocity(robotVel, flywheelVelocity, turretPos);

            double theta = ProjectileMathWithDrag.solveTheta(
                    targetDisplacements,
                    initialVelocity
            );

            confidence = ProjectileMathWithDrag.getConfidence(
                    targetDisplacements,
                    initialVelocity,
                    theta
            );

            if (!Double.isNaN(theta) && confidence > 0.5) {
                hoodPos = servoPosFromRad(
                        theta,
                        HoodServo.HOOD_MIN_RAD,
                        HoodServo.HOOD_MAX_RAD,
                        HoodServo.HOOD_MIN_POS,
                        HoodServo.HOOD_MAX_POS
                );
            }
        }
    }

    private double angleTurretTo(Pose currentPos, Pose targetPos) {
        Vector offset = new Vector();
        offset.setOrthogonalComponents(targetPos.getX() - currentPos.getX(), targetPos.getY() - currentPos.getY());
        double targetAngle = offset.getTheta() + Math.PI;
        double currentAngle = follower.getPose().getHeading();
        double deltaAngle = Angle.normalizeAnglePi(targetAngle - currentAngle);
        return Range.clip(deltaAngle, -TurretMotor.RAD_LIMIT, TurretMotor.RAD_LIMIT);
    }

    private double calcInitialLaunchVelocity(Vector robotVelocity, double flywheelVelocity, double turretAngleOffset) {
        double robotSpeed = robotVelocity.getMagnitude() * Math.cos(turretAngleOffset);
        return robotSpeed + flywheelVelocity;
    }
}
