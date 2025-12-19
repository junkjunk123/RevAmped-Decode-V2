package org.firstinspires.ftc.teamcode.revamped.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Race;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.revamped.utils.FollowParameters;
import org.firstinspires.ftc.teamcode.revamped.utils.PathSupplier;

import java.util.Iterator;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

public class Drivetrain {
    public final Follower follower;
    private final boolean isTeleOp;
    private Iterator<FollowParameters> paths;
    public static Pose startPose;
    private final List<DcMotorEx> motors;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;
    private boolean prevVelZero;
    private boolean fieldCentric;
    public static float MAGNITUDE_ZERO = 0.15f;
    public static float ANGLE_ZERO = (float) Math.tan(Math.toRadians(15));
    public static float MAX_POWER = 1.0f;

    public Drivetrain(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        motors = ((Mecanum) follower.drivetrain).getMotors();
        apply(m -> m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
        initMotors();
        isTeleOp = true;
    }

    public Drivetrain(HardwareMap hardwareMap, PathSupplier paths) {
        follower = Constants.createFollower(hardwareMap);
        this.paths = paths.paths(follower).iterator();
        startPose = paths.startPose();
        follower.setStartingPose(startPose);
        motors = ((Mecanum) follower.drivetrain).getMotors();
        initMotors();
        isTeleOp = false;
    }

    private void initMotors() {
        leftFront = motors.get(0);
        leftRear = motors.get(1);
        rightRear = motors.get(2);
        rightFront = motors.get(3);
    }

    public void followNext() {
        if (paths.hasNext()) {
            paths.next().follow(follower);
        }
    }

    public Command followNext(Function<Drivetrain, Boolean> isDone) {
        return new Command()
                .setStart(this::followNext)
                .setDone(() -> isDone.apply(this));
    }

    public Race followNext(Function<Drivetrain, Boolean> isDone, double timeout) {
        return new Race(
                followNext(isDone),
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
        return follower.getVelocity().getMagnitude() < follower.getCurrentPath().getPathEndVelocityConstraint();
    }

    public boolean velocityCondition(double dist) {
        return !follower.isBusy() || (distanceFromTarget() < dist && velocityCondition());
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
        if (!isTeleOp)
            follower.update();
    }

    public void updateLocalization() {
        follower.poseTracker.update();
    }
}
