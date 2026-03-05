package org.firstinspires.ftc.teamcode.utils.hardware;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.calc.Integrator;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channels;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Speaker;

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
    }

    public HwCRServo(HardwareMap hardwareMap, Encoder encoder, String id) {
        this.hardware = new CRServoImplEx[] {HwDevice.init(hardwareMap, CRServoImplEx.class, id)};
        this.id = id;
        this.encoder = encoder;
        resetPosition();
    }

    public HwCRServo(HardwareMap hardwareMap, String... ids) {
        this.hardware = Arrays.stream(ids)
                .map(s -> HwDevice.init(hardwareMap, CRServoImplEx.class, s))
                .toArray(CRServoImplEx[]::new);
        this.id = Arrays.toString(ids);
    }

    public HwCRServo(HardwareMap hardwareMap, Encoder encoder, String... ids) {
        this.hardware = Arrays.stream(ids)
                .map(s -> HwDevice.init(hardwareMap, CRServoImplEx.class, s))
                .toArray(CRServoImplEx[]::new);
        this.id = Arrays.toString(ids);
        if (encoder == null) throw new IllegalArgumentException("Encoder cannot be null");
        this.encoder = encoder;
        resetPosition();
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
        for (CRServoImplEx servo : hardware)
            servo.setDirection(direction);
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

    protected void setEncoderBase(int encoderBase) {
        this.encoderBase = encoderBase;
    }

    public Encoder getEncoder() {
        return encoder;
    }

    public void invalidateCache() {
        currentPos = null;
    }

    public void deenergize() {
        for (CRServoImplEx servo : hardware) {
            servo.setPwmDisable();
        }
    }

    public Speaker<String> test() {
        setPower(0.5);
        Integrator encoder = new Integrator();
        return new Speaker<>(c ->
                new Sequential(
                        new Race(
                                new Wait(5000),
                                new Infinite(() -> {
                                    encoder.update(Math.abs(getVelocity()));
                                })
                        ),
                        Channels.send(c, () -> {
                            if (encoder.getIntegral() > 15)
                                return id + "MOTOR TEST PASS: Encoder counts normal.";
                            else
                                return id + "MOTOR TEST FAIL: Encoder counts too low!";
                        })
                )
        );
    }

    @NonNull
    @Override
    public String toString() {
        return id;
    }
}