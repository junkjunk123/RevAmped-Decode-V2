package org.firstinspires.ftc.teamcode.utils.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class HwCRServo implements HwDevice {
    private double lastPower = 0;
    public final CRServoImplEx[] hardware;
    private double powerThreshold = 0.01;
    private Integer currentPos;
    private int encoderBase;
    private final String id;
    private Encoder encoder;

    public HwCRServo(HardwareMap hardwareMap, String id) {
        this.hardware = new CRServoImplEx[] {HwDevice.init(hardwareMap, CRServoImplEx.class, id)};
        this.id = id;
        resetPosition();
    }

    public HwCRServo(HardwareMap hardwareMap, Encoder encoder, String id) {
        this.hardware = new CRServoImplEx[] {HwDevice.init(hardwareMap, CRServoImplEx.class, id)};
        this.id = id;
        resetPosition();
        this.encoder = encoder;
    }

    public HwCRServo(HardwareMap hardwareMap, String... ids) {
        this.hardware = Arrays.stream(ids)
                .map(s -> HwDevice.init(hardwareMap, CRServoImplEx.class, s))
                .toArray(CRServoImplEx[]::new);
        this.id = Arrays.toString(ids);
        resetPosition();
    }

    public HwCRServo(HardwareMap hardwareMap, Encoder encoder, String... ids) {
        this.hardware = Arrays.stream(ids)
                .map(s -> HwDevice.init(hardwareMap, CRServoImplEx.class, s))
                .toArray(CRServoImplEx[]::new);
        this.id = Arrays.toString(ids);
        resetPosition();
        if (encoder == null) throw new IllegalArgumentException("Encoder cannot be null");
        this.encoder = encoder;
    }

    public void setPower(double power) {
        if ((Math.abs(lastPower - power) > powerThreshold) || (power == 0 && lastPower != 0)) {
            lastPower = power;

            for (CRServoImplEx motor : hardware)
                motor.setPower(power);
        }
    }

    public CRServoImplEx get() {
        return hardware[0];
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    private int getPosRaw() {
        return encoder.getPosition() - encoderBase;
    }

    public int getPosition() {
        return currentPos == null ? currentPos = getPosRaw() : currentPos;
    }

    public void update() {
        currentPos = null;
    }

    public void resetPosition() {
        resetPosition(0);
    }

    public void resetPosition(int pos) {
        encoderBase = getPosition() + encoderBase - pos;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        for (CRServoImplEx motor : hardware)
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

    public int getEncoderBase() {
        return encoderBase;
    }

    public void setEncoder(Encoder encoder) {
        this.encoder = encoder;
    }

    @NonNull
    @Override
    public String toString() {
        return id;
    }
}