package org.firstinspires.ftc.teamcode.revamped.math.calc;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A class that handles calculations for integral, derivative, and average values.
 * It uses an ElapsedTime instance to measure time intervals for these calculations.
 */
public class Integrator {
    private double integral = 0;
    private int count = 0;
    private double prevVal = 0;
    private double derivative = 0;
    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Updates the integral, derivative, and previous value based on the new value provided.
     * @param newVal the new value to update the calculations with
     */
    public void update(double newVal) {
        double dt = timer.nanoseconds() / Math.pow(10.0, 9.0);
        timer.reset();
        integral += newVal * dt;
        derivative = (newVal - prevVal) / dt;
        prevVal = newVal;
        count++;
    }

    /**
     * Returns the current value that was set in the handler.
     * @return the current value
     */
    public double getVal() {
        return prevVal;
    }

    /**
     * Returns the current integral value.
     * @return the integral value
     */
    public double getIntegral() {
        return integral;
    }

    /**
     * Returns the current derivative value.
     * @return the derivative value
     */
    public double getDerivative() {
        return derivative;
    }

    /**
     * Resets the integral, previous value, derivative, timer, and count to their initial states.
     */
    public void reset() {
        integral = 0;
        prevVal = 0;
        derivative = 0;
        timer.reset();
        count = 0;
    }

    /**
     * Returns the count of updates made to the handler.
     * @return the count of updates
     */
    public int getCount() {
        return count;
    }

    /**
     * Sets the count of updates made to the handler.
     * @param count the new count of updates
     */
    public void setCount(int count) {
        this.count = count;
    }

    /**
     * Returns the average value of the integral based on the count of updates.
     * @return the average value
     */
    public double getAvg() {
        return integral / count;
    }
}
