package org.firstinspires.ftc.teamcode.utils.math.calc;

import com.pedropathing.control.KalmanFilterParameters;

public class SingleStateKalman {
    private final KalmanFilterParameters parameters;
    private double state;
    private double variance;

    public SingleStateKalman(KalmanFilterParameters parameters) {
        this.parameters = parameters;
        reset();
    }

    public SingleStateKalman(KalmanFilterParameters parameters, double startState, double startVariance) {
        this.parameters = parameters;
        reset(startState, startVariance);
    }

    public void reset() {
        reset(0.0, 1.0);
    }

    public void reset(double startState, double startVariance) {
        state = startState;
        variance = startVariance;
    }

    /**
     * Update the filter with a new measurement
     *
     * @param measurement The new measured value
     */
    public void update(double measurement) {
        // Prediction step
        variance += parameters.modelCovariance;

        // Measurement update
        double kalmanGain = variance / (variance + parameters.dataCovariance);
        state += kalmanGain * (measurement - state);
        variance *= (1.0 - kalmanGain);
    }

    public double getState() {
        return state;
    }

    public double getVariance() {
        return variance;
    }
}
