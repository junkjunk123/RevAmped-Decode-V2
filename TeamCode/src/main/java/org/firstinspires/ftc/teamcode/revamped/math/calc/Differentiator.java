package org.firstinspires.ftc.teamcode.revamped.math.calc;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.revAmped.components.hardware.Encoder;

import java.util.function.Supplier;

public class Differentiator {
    private Double previousValue;
    private final ElapsedTime timer;
    private final Supplier<Double> optional;
    private Double cachedDerivative;
    private final Supplier<Double> currentValue;

    public Differentiator(Supplier<Double> optional, Supplier<Double> currentValue) {
        this.currentValue = currentValue;
        this.timer = new ElapsedTime();
        this.timer.reset();
        this.previousValue = null;
        this.optional = optional;
    }

    public Differentiator(Encoder encoder) {
        this(encoder::getVelocity,() -> (double) encoder.getPosition());
    }

    public double calculate() {
        if (cachedDerivative != null)
            return cachedDerivative;

        if (previousValue == null) {
            previousValue = currentValue.get();
            timer.reset();
            return optional.get(); // No previous value to compare to
        }

        double deltaTime = timer.seconds();
        if (deltaTime == 0) {
            return 0; // Prevent division by zero
        }

        double currentValue = this.currentValue.get();
        double derivative = (currentValue - previousValue) / deltaTime;
        previousValue = currentValue;
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
