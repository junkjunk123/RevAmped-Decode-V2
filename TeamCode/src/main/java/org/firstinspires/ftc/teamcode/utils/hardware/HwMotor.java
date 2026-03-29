package org.firstinspires.ftc.teamcode.utils.hardware;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.math.calc.Integrator;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channels;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Speaker;

import java.util.Arrays;

public class HwMotor implements HwDevice {
    private double lastPower = 0;
    public final DcMotorEx[] hardware;
    private double powerThreshold = 0.01;
    private Integer currentPos;
    private int encoderBase;
    private final String id;
    private Encoder encoder;

    public HwMotor(HardwareMap hardwareMap, String id) {
        this(hardwareMap, true, id);
    }

    public HwMotor(HardwareMap hardwareMap, boolean initEncoder, String id) {
        this.hardware = new DcMotorEx[] {HwDevice.init(hardwareMap, DcMotorEx.class, id)};
        this.id = id;
        this.encoder = initEncoder ? Encoder.fromMotor(get()) : null;
        if (encoder != null) resetPosition();
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public HwMotor(HardwareMap hardwareMap, String... ids) {
        this(hardwareMap, true, ids);
    }

    public HwMotor(HardwareMap hardwareMap, boolean initEncoder, String... ids) {
        this.hardware = Arrays.stream(ids).map(s -> HwDevice.init(hardwareMap, DcMotorEx.class, s)).toArray(DcMotorEx[]::new);
        this.id = Arrays.toString(ids);
        this.encoder = initEncoder ? Encoder.fromMotor(get()) : null;
        if (encoder != null) resetPosition();
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        for (DcMotorEx motor : hardware) {
            motor.setPower(0);
            motor.setMotorDisable();
        }
    }

    public Speaker<String> test() {
        setPower(0.5);
        Integrator current = new Integrator();
        Integrator encoder = new Integrator();
        double nominalCurrent = get().getCurrent(CurrentUnit.AMPS);
        return new Speaker<>(c ->
                new Sequential(
                        new Race(
                                new Wait(5000),
                                new Infinite(() -> {
                                    current.update(Math.abs(nominalCurrent - get().getCurrent(CurrentUnit.AMPS)));
                                    encoder.update(Math.abs(this.encoder.getVelocity()));
                                })
                        ),
                        new Instant(() -> setPower(0)),
                        Channels.send(c, () -> {
                            if (current.getIntegral() > 0.08)
                                return id + "MOTOR TEST PASS: Current draw normal.";
                            else
                                return id + "MOTOR TEST FAIL: Current draw too low!";
                        }),
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