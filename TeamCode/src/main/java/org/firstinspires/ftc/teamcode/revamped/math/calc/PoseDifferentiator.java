package org.firstinspires.ftc.teamcode.revamped.math.calc;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

public class PoseDifferentiator {
    private Pose previousValue;
    private final ElapsedTime timer;
    private final Supplier<Pose> optional;
    private Pose cachedDerivative;
    private final Supplier<Pose> currentValue;

    public PoseDifferentiator(Supplier<Pose> optional, Supplier<Pose> currentValue) {
        this.currentValue = currentValue;
        this.timer = new ElapsedTime();
        this.timer.reset();
        this.previousValue = null;
        this.optional = optional;
    }

    public Pose calculate() {
        return calculate(currentValue.get());
    }

    public Pose calculate(Pose currentPose) {
        if (cachedDerivative != null) return cachedDerivative;

        if (previousValue == null) {
            previousValue = currentPose;
            timer.reset();
            return optional.get(); // No previous value to compare to
        }

        double deltaTime = timer.seconds();
        if (deltaTime == 0) {
            return new Pose(); // Prevent division by zero
        }

        Pose derivative = (currentPose.minus(previousValue)).div(deltaTime);
        previousValue = currentPose;
        timer.reset();
        cachedDerivative = derivative;
        return derivative;
    }

    public void reset() {
        previousValue = null;
        cachedDerivative = null;
        timer.reset();
    }

    public void update() {
        cachedDerivative = null;
    }
}
