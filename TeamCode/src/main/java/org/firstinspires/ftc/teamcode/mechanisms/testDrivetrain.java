package org.firstinspires.ftc.teamcode.revamped.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class testDrivetrain {
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;

    private boolean prevVelZero = false;

    public static float MAGNITUDE_ZERO = 0.15f;
    public static float ANGLE_ZERO = (float) Math.tan(Math.toRadians(15));
    public static float MAX_POWER = 1.0f;

    public testDrivetrain(HardwareMap hardwareMap) {
        // Initialize motors from hardware map
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Set all motors to brake when power is zero
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run using encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);    }

    /**
     * Sets the power levels for each of the four motors.
     *
     * @param lf the power level for the left front motor (-1.0 to 1.0)
     * @param lr the power level for the left rear motor (-1.0 to 1.0)
     * @param rf the power level for the right front motor (-1.0 to 1.0)
     * @param rr the power level for the right rear motor (-1.0 to 1.0)
     */
    public void setPowers(double lf, double lr, double rf, double rr) {
        // Optimization: only update motors if power has changed
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
     * Arcade drive method for controlling the robot using gamepad input.
     *
     * @param x the x-value from the gamepad (strafe)
     * @param y the y-value from the gamepad (forward/backward)
     * @param rx the rotation value from the gamepad
     * @param deadzone deadzone value for joystick input
     * @param angleZero the angle threshold for cardinal direction snapping
     * @param maxPower the maximum power for the motors
     */
    public void arcadeDrive(double x, double y, double rx, float deadzone, float angleZero, float maxPower) {
        // Check if any input exceeds deadzone
        if (Math.abs(y) > deadzone || Math.abs(x) > deadzone || Math.abs(rx) > deadzone) {
            // Apply smoothing to gamepad angles
            double[] smoothed = smoothGamepadAngle(x, y, angleZero);

            // Apply tangent curve for more precise control at low speeds
            if (Math.abs(y) > deadzone) {
                y = -0.5 * Math.tan(smoothed[1] * 1.12);
            } else {
                y = 0;
            }

            if (Math.abs(x) > deadzone) {
                x = 0.5 * Math.tan(smoothed[0] * 1.12);
            } else {
                x = 0;
            }

            if (Math.abs(rx) > deadzone) {
                rx = 0.5 * Math.tan(rx * 1.12);
            } else {
                rx = 0;
            }

            // Calculate mecanum drive motor powers
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPow = (y + x + rx) / denominator;
            double backLeftPow = (y - x + rx) / denominator;
            double frontRightPow = (y - x - rx) / denominator;
            double backRightPow = (y + x - rx) / denominator;

            // Normalize powers to max power limit
            double highestPower = Math.max(
                    Math.max(Math.abs(frontLeftPow), Math.abs(backLeftPow)),
                    Math.max(Math.abs(frontRightPow), Math.abs(backRightPow))
            );

            if (highestPower > maxPower) {
                frontLeftPow *= maxPower / highestPower;
                frontRightPow *= maxPower / highestPower;
                backRightPow *= maxPower / highestPower;
                backLeftPow *= maxPower / highestPower;
            }

            setPowers(frontLeftPow, backLeftPow, frontRightPow, backRightPow);
        } else if (!prevVelZero) {
            // Stop motors if no input and they weren't already stopped
            setPowers(0, 0, 0, 0);
        }
    }

    /**
     * Simplified arcade drive with default parameters.
     */
    public void arcadeDrive(double x, double y, double rx) {
        arcadeDrive(x, y, rx, MAGNITUDE_ZERO, ANGLE_ZERO, MAX_POWER);
    }

    /**
     * Smooths gamepad input to snap to cardinal directions.
     *
     * @param x the x-value
     * @param y the y-value
     * @param zeroAngle the angle threshold for snapping
     * @return array with smoothed [x, y] values
     */
    private double[] smoothGamepadAngle(double x, double y, double zeroAngle) {
        if (x == 0 || y == 0) {
            return new double[]{x, y};
        } else if (Math.abs(y / x) < zeroAngle) {
            return new double[]{x, 0};
        } else if (Math.abs(x / y) < zeroAngle) {
            return new double[]{0, y};
        } else {
            return new double[]{x, y};
        }
    }

    /**
     * Stops all motors.
     */
    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    /**
     * Get individual motor for direct access if needed.
     */
    public DcMotorEx getLeftFront() { return leftFront; }
    public DcMotorEx getRightFront() { return rightFront; }
    public DcMotorEx getLeftRear() { return leftRear; }
    public DcMotorEx getRightRear() { return rightRear; }
}