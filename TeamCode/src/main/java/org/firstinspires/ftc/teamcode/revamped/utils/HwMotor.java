package org.firstinspires.ftc.teamcode.revamped.utils;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.revamped.math.calc.Differentiator;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.HwDevice;

public class HwMotor implements HwDevice {
    private double lastPower = 0;
    public final DcMotorEx hardware;
    private double powerThreshold = 0.01;
    private Integer currentPos;
    private int encoderBase;
    private final Differentiator velocityCalculator;
    private final String id;

    public HwMotor(HardwareMap hardwareMap, String id) {
        this.hardware = HwDevice.init(hardwareMap, DcMotorEx.class, id);
        this.id = id;
        velocityCalculator = new Differentiator(() -> 0.0, () -> (double) getPosition());
    }

    public void setPower(double power) {
        if ((Math.abs(this.lastPower - power) > this.powerThreshold) || (power == 0 && lastPower != 0)) {
            lastPower = power;
            hardware.setPower(power);
        }
    }

    public double getVelocity() {
        return velocityCalculator.calculate();
    }

    private int getPosRaw() {
        return currentPos - encoderBase;
    }

    public int getPosition() {
        return currentPos == null ? currentPos = getPosRaw() : currentPos;
    }

    public void update() {
        currentPos = null;
        velocityCalculator.update();
    }

    public void resetPosition() {
        encoderBase = getPosition();
    }

    public void resetPosition(int pos) {
        encoderBase = getPosition() - pos;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.hardware.setDirection(direction);
    }

    public void setCachingThreshold(double powerThreshold) {
        this.powerThreshold = powerThreshold;
    }

    public double getPower() {
        return lastPower;
    }

    public void setMode(DcMotor.RunMode runMode) {
        this.hardware.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        this.hardware.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @NonNull
    @Override
    public String toString() {
        return id;
    }
}