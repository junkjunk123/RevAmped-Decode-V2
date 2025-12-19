package org.firstinspires.ftc.teamcode.utils.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class HwMotor implements HwDevice {
    private double lastPower = 0;
    public final DcMotorEx[] hardware;
    private double powerThreshold = 0.01;
    private Integer currentPos;
    private int encoderBase;
    private final String id;

    public HwMotor(HardwareMap hardwareMap, String id) {
        this.hardware = new DcMotorEx[] {HwDevice.init(hardwareMap, DcMotorEx.class, id)};
        this.id = id;
    }

    public HwMotor(HardwareMap hardwareMap, String... ids) {
        this.hardware = Arrays.stream(ids).map(s -> HwDevice.init(hardwareMap, DcMotorEx.class, s)).toArray(DcMotorEx[]::new);
        this.id = Arrays.toString(ids);
    }

    public void setPower(double power) {
        if ((Math.abs(lastPower - power) > powerThreshold) || (power == 0 && lastPower != 0)) {
            lastPower = power;

            for (DcMotorEx motor : hardware)
                motor.setPower(power);
        }
    }

    public DcMotorEx get() {
        return hardware[0];
    }

    public double getVelocity() {
        return hardware[0].getVelocity();
    }

    private int getPosRaw() {
        return hardware[0].getCurrentPosition() - encoderBase;
    }

    public int getPosition() {
        return currentPos == null ? currentPos = getPosRaw() : currentPos;
    }

    public void update() {
        currentPos = -1;
    }

    public void resetPosition() {
        encoderBase = getPosition();
    }

    public void resetPosition(int pos) {
        encoderBase = getPosition() - pos;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        for (DcMotorEx motor : hardware)
            motor.setDirection(direction);
    }

    public void setCachingThreshold(double powerThreshold) {
        this.powerThreshold = powerThreshold;
    }

    public double getPower() {
        return lastPower;
    }

    public boolean atPower(float power) {
        return Math.abs(lastPower - power) <= powerThreshold;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : hardware)
            motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : hardware)
            motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public int getEncoderBase() {
        return encoderBase;
    }

    @NonNull
    @Override
    public String toString() {
        return id;
    }
}