package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.ColoredDecodePose;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.FollowParameters;
import org.firstinspires.ftc.teamcode.pedro.PathSupplier;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.math.MathUtil;
import org.firstinspires.ftc.teamcode.utils.math.calc.Vector2D;

import java.util.ArrayDeque;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

public class Drivetrain {
    public final Follower follower;
    private ArrayDeque<FollowParameters> paths;
    //20.5+ X -20 Y OFFSETS NO OFFSETS APPLIED RNZ
    public static Pose startPose = new Pose(32, 134, Math.toRadians(-90));
    public static ColoredDecodePose resetPose = new ColoredDecodePose(19,84.5,Math.toRadians(180));
    private final List<DcMotorEx> motors;
    public final DcMotorEx leftFront;
    public final DcMotorEx rightFront;
    public final DcMotorEx leftRear;
    public final DcMotorEx rightRear;
    private boolean prevVelZero;
    private boolean fieldCentric;
    public static float MAGNITUDE_ZERO = 0.15f;
    public static float ANGLE_ZERO = (float) Math.tan(Math.toRadians(15));
    public static float MAX_POWER = 1.0f;
    public boolean canShoot = true;
    private boolean holdingPose;
    public static double CLAMP_POWER = 0;
    public static double MAX_LAMBDA = 0.95;
    public static double MAX_VELOCITY = 76;
    public static boolean tipCorrection = false;
    private boolean isDoneFollowing = true;
    public static Function<Drivetrain, Boolean> isDone = d -> d.velocityCondition(4);

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.createFollowerTeleOp(hardwareMap);
        follower.setStartingPose(startPose);
        motors = ((Mecanum) follower.drivetrain).getMotors();
        leftFront = motors.get(0);
        leftRear = motors.get(1);
        rightFront = motors.get(2);
        rightRear = motors.get(3);
        Globals.isTeleOp = true;
        follower.startTeleOpDrive();
        follower.update();
    }

    public Drivetrain(HardwareMap hardwareMap, PathSupplier paths) {
        follower = Constants.createFollower(hardwareMap);
        this.paths = new ArrayDeque<>(paths.paths(follower));
        follower.setStartingPose(paths.startPose());
        motors = ((Mecanum) follower.drivetrain).getMotors();
        leftFront = motors.get(0);
        leftRear = motors.get(1);
        rightFront = motors.get(2);
        rightRear = motors.get(3);
        Globals.isTeleOp = false;
        follower.update();
    }

    public void followNext() {
        if (!paths.isEmpty()) {
            paths.poll().follow(follower);
        } else {
            BezierPoint point = new BezierPoint(follower.getPose());
            Path path = new Path(point);
            path.setConstantHeadingInterpolation(follower.getHeading());
            follower.followPath(path);
        }
    }

    public void followNext(double brakingStrength) {
        if (!paths.isEmpty()) {
            paths.poll().follow(follower, brakingStrength);
        } else {
            BezierPoint point = new BezierPoint(follower.getPose());
            Path path = new Path(point);
            path.setConstantHeadingInterpolation(follower.getHeading());
            follower.followPath(path);
        }
    }

    public void followLast() {
        if (!paths.isEmpty()) {
            paths.pollLast().follow(follower);
        } else {
            BezierPoint point = new BezierPoint(follower.getPose());
            Path path = new Path(point);
            path.setConstantHeadingInterpolation(follower.getHeading());
            follower.followPath(path);
        }
    }

    public void followLast(double brakingStrength) {
        if (!paths.isEmpty()) {
            paths.pollLast().follow(follower, brakingStrength);
        } else {
            BezierPoint point = new BezierPoint(follower.getPose());
            Path path = new Path(point);
            path.setConstantHeadingInterpolation(follower.getHeading());
            follower.followPath(path);
        }
    }

    public ICommand holdPose() {
        return new Sequential(
                new Instant(() -> {
                    follower.update();
                    BezierPoint point = new BezierPoint(follower.getPose());
                    Path path = new Path(point);
                    path.setConstantHeadingInterpolation(follower.getHeading());
                    follower.followPath(path);
                    canShoot = false;
                    holdingPose = true;
                }),
                new Wait(350),
                new Instant(() -> canShoot = true)
        );
    }

    public void stopHoldPose() {
        if (!holdingPose) return;
        holdingPose = false;
        follower.startTeleOpDrive();
        follower.update();
    }

    public void skip(int i) {
        for (int j = 0; j < i; j++) paths.poll();
    }

    public Command followNext(Function<Drivetrain, Boolean> isDone) {
        return new Command()
                .setStart(() -> {
                    followNext();
                    isDoneFollowing = false;
                })
                .setDone(() -> isDone.apply(this))
                .setEnd(c -> isDoneFollowing = true);
    }

    public Command followLast(Function<Drivetrain, Boolean> isDone) {
        return new Command()
                .setStart(() -> {
                    followLast();
                    isDoneFollowing = false;
                })
                .setDone(() -> isDone.apply(this))
                .setEnd(c -> isDoneFollowing = true);
    }

    public Race followNext(Function<Drivetrain, Boolean> isDone, double timeout) {
        return new Race(
                followNext(isDone),
                new Wait(timeout)
        );
    }

    public Race followNext(Function<Drivetrain, Boolean> isDone, double timeout, double brakingStrength) {
        return new Race(
                followNext(isDone),
                new Wait(timeout)
        );
    }

    public ICommand follow() {
        return followNext(d -> d.velocityCondition(4), 3000);
    }

    public ICommand follow(double brakingStrength) {
        return new Race(
                new Command()
                        .setStart(() -> {followNext(brakingStrength); isDoneFollowing = false;})
                        .setDone(() -> isDone.apply(this))
                        .setEnd(c -> isDoneFollowing = true),
                new Wait(3000)
        );
    }

    public void skip() {
        paths.poll();
    }

    public Race followLast(Function<Drivetrain, Boolean> isDone, double timeout) {
        return new Race(
                followLast(isDone),
                new Wait(timeout)
        );
    }

    /**
     * The robot's distance from the target end position
     * @return the distance to the target position
     */
    public double distanceFromTarget() {
        return follower.getPose().distanceFrom(follower.getCurrentPathChain().endPoint());
    }

    public boolean velocityCondition() {
        return follower.getVelocity().dot(follower.getClosestPointTangentVector().normalize())
                < follower.getCurrentPath().getPathEndVelocityConstraint();
    }

    public boolean velocityCondition(double dist) {
        return !follower.isBusy() || (distanceFromTarget() < dist && velocityCondition());
    }

    public boolean tValueCondition(double t) {
        return follower.getCurrentTValue() > t;
    }

    public void apply(Consumer<DcMotorEx> action) {
        for (DcMotorEx motor : motors)
            action.accept(motor);
    }

    /**
     * Sets the power levels for each of the four motors in the system.
     *
     * @param lf the power level for the left front motor, must be between -1.0 and 1.0
     * @param lr the power level for the left rear motor, must be between -1.0 and 1.0
     * @param rf the power level for the right front motor, must be between -1.0 and 1.0
     * @param rr the power level for the right rear motor, must be between -1.0 and 1.0
     */

    public void setPowers(double lf, double lr, double rf, double rr) {
        if (lf == lr && lr == rf && rf == rr && lf == 0) {
            if (prevVelZero) {
                return;
            }

            prevVelZero = true;
        } else {
            prevVelZero = false;
        }

        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightFront.setPower(rf);
        rightRear.setPower(rr);
    }

    /**
     * Arcade drive method for controlling the robot using a gamepad.
     * @param x the x-value from the gamepad
     * @param y the y-value from the gamepad
     * @param rx the rotation value from the gamepad
     * @param DZ deadzone value for joystick input
     * @param angleZero the angle at which the gamepad is considered to be at zero
     * @param maxPower the maximum power for the motors
     */
    public void arcadeDrive(double x, double y, double rx, float DZ, float angleZero, float maxPower) {
        double robotHeading;

        if (Math.abs(y) > DZ || Math.abs(x) > DZ || Math.abs(rx) > DZ) {
            Pose movementVector = smoothGamepadAngle(x, y, angleZero);

            if (Math.abs(y) > DZ) {
                y = -0.5 * Math.tan(movementVector.getY() * 1.12); // Remember, this is reversed!
            } else {
                y = 0;
            }

            if (Math.abs(x) > DZ) {
                x = 0.5 * Math.tan(movementVector.getX() * 1.12);
            } else {
                x = 0;
            }

            if (Math.abs(rx) > DZ) {
                rx = 0.5 * Math.tan(rx * 1.12);
            } else {
                rx = 0;
            }

            if (fieldCentric) {
                robotHeading = follower.getHeading();
                x = x * Math.cos(-robotHeading) - y * Math.sin(-robotHeading);
                y = x * Math.sin(-robotHeading) + y * Math.cos(robotHeading);
            }

            if (tipCorrection) {
                Localizer octoquad = follower.poseTracker.getLocalizer();
                double heading = octoquad.getPose().getHeading();
                Vector voltage = new Vector2D(x, y);
                Vector vel = octoquad.getVelocityVector();
                Vector robotVel = vel.transform(MathUtil.rotMatrix(heading));
                robotVel = new Vector2D(robotVel.getYComponent(), -robotVel.getXComponent());
                Globals.telemetry.addData("robotVel", Vector2D.print(robotVel));
                Globals.telemetry.addData("voltage", Vector2D.print(voltage));
                double alpha = voltage.dot(robotVel) / (voltage.getMagnitude() * robotVel.getMagnitude() + 1e-9);

                if (alpha < 0.2) {
                    Vector forwardHeadingVector = new Vector(1, heading);
                    double parallelVelNormalized = Math.abs(vel.dot(forwardHeadingVector)) / MAX_VELOCITY * 2;
                    double lambda = getLambda(parallelVelNormalized, alpha);
                    Vector e2 = new Vector2D(0, 1);
                    Vector forwardVoltage = voltage.projectOnto(e2);
                    Vector perpendicularVoltage = voltage.minus(forwardVoltage);
                    Vector adjustedVoltage = forwardVoltage.times(lambda).plus(perpendicularVoltage);
                    x = adjustedVoltage.getXComponent();
                    y = adjustedVoltage.getYComponent();
                }
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPow = (y + x + rx) / denominator;
            double backLeftPow = (y - x + rx) / denominator;
            double frontRightPow = (y - x - rx) / denominator;
            double backRightPow = (y + x - rx) / denominator;

            double highestPower = Math.max(Math.max(frontLeftPow, backLeftPow), Math.max(frontRightPow, backRightPow));

            if (highestPower > maxPower) {
                frontLeftPow *= maxPower / highestPower;
                frontRightPow *= maxPower / highestPower;
                backRightPow *= maxPower / highestPower;
                backLeftPow *= maxPower / highestPower;
            }

            setPowers(frontLeftPow, backLeftPow, frontRightPow, backRightPow);
        } else if (!prevVelZero){
            setPowers(0,0,0,0);
        }
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    /**
     * Smooths the gamepad angle to meet human standards
     * @param x the x-value
     * @param y the y-value
     * @param zeroAngle the angle at which the gamepad is considered to be at zero
     * @return a Vector2D representing the smoothed x and y values
     */
    private Pose smoothGamepadAngle(double x, double y, double zeroAngle) {
        if (x == 0 || y==0) {
            return new Pose(x,y);
        } else if (Math.abs(y/x) < zeroAngle) {
            return new Pose(x, 0);
        } else if (Math.abs(x/y) < zeroAngle) {
            return new Pose(0,y);
        } else {
            return new Pose(x,y);
        }
    }

    public void arcadeDrive(double x, double y, double rx) {
        arcadeDrive(x, y, rx, MAGNITUDE_ZERO, ANGLE_ZERO, MAX_POWER);
    }

    public void arcadeDrive(Gamepad gamepad) {
        arcadeDrive(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x);
    }

    public void update() {
        if (!Globals.isTeleOp || isHoldingPose()) follower.update();
        else follower.poseTracker.getLocalizer().update();
    }

    public void updateLocalization() {
        follower.poseTracker.update();
    }

    public void setPower(double power) {
        apply(m -> m.setPower(power));
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        leftFront.setZeroPowerBehavior(behavior);
        leftRear.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightRear.setZeroPowerBehavior(behavior);
    }

    public boolean isHoldingPose() {
        return holdingPose;
    }

    public double getLambda(double s, double alpha) {
        Globals.telemetry.addData("s",s);
        Globals.telemetry.addData("alpha", alpha);
        double g = (1 - alpha)/2;
        Globals.telemetry.addData("g", g);
        double lambda = 1 - (1 - CLAMP_POWER) * s * s * g * g;
        Globals.telemetry.addData("lambda", lambda);
        if (lambda > MAX_LAMBDA) return 1.0;
        return Math.max(lambda, 0);
    }

    public boolean isDoneFollowing() {
        return isDoneFollowing;
    }

    public Path hold() {
        BezierPoint point = new BezierPoint(follower.getPose());
        Path path = new Path(point);
        path.setConstantHeadingInterpolation(follower.getHeading());
        return path;
    }

    public ArrayDeque<FollowParameters> getPaths() {
        return paths;
    }

    public void holdCurrentPose() {
        follower.followPath(hold());
    }
}
