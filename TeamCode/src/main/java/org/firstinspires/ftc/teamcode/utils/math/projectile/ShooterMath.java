package org.firstinspires.ftc.teamcode.utils.math.projectile;

import static org.firstinspires.ftc.teamcode.utils.math.calc.Angle.normalizeAnglePi;
import static org.firstinspires.ftc.teamcode.utils.math.calc.Angle.servoPosFromRad;
import static org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret.RAD_LIMIT;
import static org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret.TICKS_LIMIT;
import static org.firstinspires.ftc.teamcode.utils.Globals.allianceColor;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.math.MathUtil;
import org.firstinspires.ftc.teamcode.utils.math.RobotKinematicsCalculator;
import org.firstinspires.ftc.teamcode.utils.math.calc.Vector3D;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;

@Config
public class ShooterMath {
    private final Follower follower;
    public static Pose APRIL_TAG_POSE_RED;
    public static Pose APRIL_TAG_POSE_BLUE;
    public static double LAUNCH_ZONE_HEIGHT = 58;

    public static double blueX = 9.5;
    public static double blueY = 135;
    public static double redX = 122;
    public static double redY = 120;

    public static double BALL_LAUNCH_MS = 100; //time through flywheel

    public static boolean velocityCompensation;

    private int turretPos;
    private double hoodPos;
    private double confidence;

    public ShooterMath(Follower follower) {
        this.follower = follower;
        APRIL_TAG_POSE_BLUE = new Pose(blueX, blueY);
        APRIL_TAG_POSE_RED = new Pose(redX, redY);
    }

    public void update(boolean trackTurret, boolean trackHood, double flywheelVelocity) {
        if (trackHood || trackTurret) {
            Pose targetPos = allianceColor == AllianceColor.Red ? APRIL_TAG_POSE_RED : APRIL_TAG_POSE_BLUE;
            Pose currentPos = follower.getPose();
            Vector robotVelocity = null;
            Pose projectedRobotPose = null;
            if (velocityCompensation) {
                Matrix inverseRotation = MathUtil.rotMatrix(currentPos.getHeading()).transposed();
                Vector robotLinearVel = follower.getVelocity().transform(inverseRotation);
                Pose robotVelPose = new Pose(robotLinearVel.getXComponent(), robotLinearVel.getYComponent(), robotLinearVel.getTheta());
                robotVelocity = robotVelPose.getAsVector();
                projectedRobotPose = RobotKinematicsCalculator.getProjectedPoseWithConstantVelocity(
                        currentPos,
                        BALL_LAUNCH_MS / 1000.0,
                        robotVelPose
                );
            }
            double deltaAngle;

            if (trackTurret) {
                if (!velocityCompensation) {
                    deltaAngle = angleTurretTo(currentPos, targetPos);
                    turretPos = (int) Range.clip(deltaAngle * TICKS_LIMIT / RAD_LIMIT, -TICKS_LIMIT, TICKS_LIMIT);
                } else {
                    Vector offset = targetPos.minus(projectedRobotPose).getAsVector();
                    Vector iHat = offset.normalize();
                    Vector jHat = new Vector();
                    jHat.setOrthogonalComponents(-iHat.getYComponent(), iHat.getXComponent());
                    Vector invertediHat = new Vector();
                    invertediHat.setOrthogonalComponents(iHat.getYComponent(), iHat.getXComponent());
                    double normalizedTurretNormalComponent = Range.clip(robotVelocity.dot(invertediHat) / flywheelVelocity, -1, 1);
                    Vector flywheelDirectionVector = new Vector();
                    flywheelDirectionVector.setOrthogonalComponents(
                            Math.sqrt(1 - Math.pow(normalizedTurretNormalComponent, 2)),
                            normalizedTurretNormalComponent
                    );
                    Matrix inverseRotMatrix = new Matrix(new double[][]{
                            {iHat.getXComponent(), iHat.getYComponent()},
                            {jHat.getXComponent(), jHat.getYComponent()}
                    });
                    double targetAngle = flywheelDirectionVector.transform(inverseRotMatrix).getTheta() + Math.PI;
                    deltaAngle = normalizeAnglePi(targetAngle - projectedRobotPose.getHeading());
                    deltaAngle = Range.clip(deltaAngle, -RAD_LIMIT, RAD_LIMIT);
                    turretPos = (int) Range.clip(deltaAngle * TICKS_LIMIT / RAD_LIMIT, -TICKS_LIMIT, TICKS_LIMIT);
                }
            }

            if (trackHood) {
                Vector3D target = Vector3D.get3DPosition(targetPos, LAUNCH_ZONE_HEIGHT);
                Pose robotPose = velocityCompensation ? projectedRobotPose : currentPos;
                Pose targetDisplacements = new Pose(
                        robotPose.distanceFrom(new Pose(target.getX(), target.getY())),
                        target.getZ() - LAUNCH_ZONE_HEIGHT
                );
                double initialVelocity = calcInitialLaunchVelocity(robotVelocity, flywheelVelocity, turretPos);

                double theta = ProjectileMathWithDrag.solveTheta(
                        targetDisplacements,
                        initialVelocity,
                        hoodPos
                );

                confidence = ProjectileMathWithDrag.getConfidence(
                        targetDisplacements,
                        initialVelocity,
                        theta
                );

                if (!Double.isNaN(theta) && confidence > 0.5) {
                    hoodPos = servoPosFromRad(
                            theta,
                            Hood.HOOD_MIN_RAD,
                            Hood.HOOD_MAX_RAD,
                            Hood.HOOD_MIN_POS,
                            Hood.HOOD_MAX_POS
                    );
                }
            }
        }
    }

    public void reset(int turretPos, float hoodPos) {
        this.turretPos = turretPos;
        this.hoodPos = hoodPos;
    }

    private double angleTurretTo(Pose currentPos, Pose targetPos) {
        Vector offset = new Vector();
        offset.setOrthogonalComponents(targetPos.getX() - currentPos.getX(), targetPos.getY() - currentPos.getY());
        double targetAngle = offset.getTheta() + Math.PI;
        double currentAngle = follower.getPose().getHeading();
        double deltaAngle = normalizeAnglePi(targetAngle - currentAngle);
        return Range.clip(deltaAngle, -RAD_LIMIT, RAD_LIMIT);
    }

    private double calcInitialLaunchVelocity(Vector robotVelocity, double flywheelVelocity, double turretAngleOffset) {
        double robotSpeed = robotVelocity.getMagnitude() * Math.cos(turretAngleOffset);
        return robotSpeed + flywheelVelocity;
    }

    public int getTurretPos() {
        return turretPos;
    }

    public double getHoodPos() {
        return hoodPos;
    }

    public double getConfidence() {
        return confidence;
    }
}