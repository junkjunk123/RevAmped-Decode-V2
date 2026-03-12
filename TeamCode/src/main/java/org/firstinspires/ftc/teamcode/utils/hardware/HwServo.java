package org.firstinspires.ftc.teamcode.utils.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.Arrays;
import java.util.function.Consumer;

public class HwServo implements HwDevice {
    public final ServoImplEx[] servo;
    private final String id;
    private double cachingTolerance = 1.0/256;
    private double lastPos = Double.NaN;

    /**
     * @param hwMap hardwareMap
     * @param id the ID of the servo as configured
     */
    public HwServo(HardwareMap hwMap, String id) {
        this.servo = new ServoImplEx[] {HwDevice.init(hwMap, ServoImplEx.class, id)};
        this.id = id;
    }

    /**
     * @param hwMap hardwareMap
     * @param id the ID of the servo as configured
     */
    public HwServo(HardwareMap hwMap, String... id) {
        servo = Arrays.stream(id).map(s -> HwDevice.init(hwMap, ServoImplEx.class, s)).toArray(ServoImplEx[]::new);
        this.id = id[0];
    }

    /**
     * Method for wrapping all writes to setPositions to the servo to check for caching tolerance
     * @param pos position requested to be written to the servo
     */
    public boolean setPosition(double pos) {
        if (Double.isNaN(lastPos) || Math.abs(pos - lastPos) >= cachingTolerance) {
            internalSetPosition(pos);
            lastPos = pos;
            return true;
        }

        return false;
    }

    private void internalSetPosition(double pos) {
        for (ServoImplEx s : servo) {
            s.setPosition(pos);
        }
    }

    private void apply(Consumer<ServoImplEx> consumer) {
        for (ServoImplEx s : servo) {
            consumer.accept(s);
        }
    }

    protected boolean evaluateCache(double pos) {
        return Double.isNaN(lastPos) || Math.abs(pos - lastPos) >= cachingTolerance;
    }

    /**
     * @return the raw position of the servo between 0 and 1
     */
    public double getPosition() {
        return servo[0].getPosition();
    }

    /**
     * @param inverted whether the servo should be inverted/reversed
     * @return this object for chaining purposes
     */
    public HwServo setInverted(boolean inverted) {
        apply(s -> s.setDirection(inverted ? Servo.Direction.REVERSE : Servo.Direction.FORWARD));
        return this;
    }

    /**
     * @return whether the servo is inverted/reversed
     */
    public boolean getInverted() {
        return servo[0].getDirection().equals(Servo.Direction.REVERSE);
    }

    /**
     * @param pwmRange the PWM range the servo should be set to
     * @return this object for chaining purposes
     */
    public HwServo setPwm(PwmControl.PwmRange pwmRange) {
        apply(servo -> getController().setServoPwmRange(servo.getPortNumber(), pwmRange));
        return this;
    }

    /**
     * @return the extended servo controller object for the servo
     */
    public ServoControllerEx getController() {
        return (ServoControllerEx) servo[0].getController();
    }

    /**
     * @return the port the servo controller is controlling the servo from
     */
    public int getPortNumber() {
        return this.servo[0].getPortNumber();
    }

    /**
     * @param cachingTolerance the new caching tolerance between servo writes
     * @return this object for chaining purposes
     */
    public HwServo setCachingTolerance(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
        return this;
    }

    /**
     * @return the caching tolerance of the servo before it writes a new power to the CR servo
     */
    public double getCachingTolerance() {
        return cachingTolerance;
    }

    public boolean atPos(float pos) {
        return Math.abs(pos - lastPos) * 256 <= 1.5;
    }

    public void deenergize() {
        apply(ServoImplEx::setPwmDisable);
    }

    @NonNull
    @Override
    public String toString() {
        return id;
    }
}