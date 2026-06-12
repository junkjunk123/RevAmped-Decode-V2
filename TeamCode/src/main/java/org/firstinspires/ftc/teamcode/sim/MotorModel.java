package org.firstinspires.ftc.teamcode.sim;

public class MotorModel {
    private double[] state; // [x, v]
    private final double kS, kV, kA, kU;
    private final AccelFunction accelCalculator;
    private int iterations;
    private double totalTime;
    private double x0, v0;
    public MotorModel(double kS, double kV, double kA, double kU, double v0, double x0) {
        if (kA == 0 || kV == 0) throw new IllegalArgumentException();
        state = new double[] {x0, v0};
        this.x0 = x0;
        this.v0 = v0;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kU = kU;
        accelCalculator = (v, u) -> (u - kS * Math.signum(v) - kV * v - kU * v * Math.max(0, -Math.signum(u * v))) / kA;
    }

    private void rungeKutta(double dt, double input) {
        double k1 = accelCalculator.calculate(state[1], input);
        double k2 = accelCalculator.calculate(state[1] + dt / 2 * k1, input);
        double k3 = accelCalculator.calculate(state[1] + dt / 2 * k2, input);
        double k4 = accelCalculator.calculate(state[1] + dt * k3, input);
        double weightedAvg = dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
        state[1] += weightedAvg;
    }

    public void update(double dt, double input) {
        iterations++;
        double prevVel = state[1];
        rungeKutta(dt, input);
        double avgVel = (state[1] + prevVel) / 2.0;
        state[0] += avgVel * dt;
    }

    public void reset() {
        state[0] = x0;
        state[1] = v0;
        iterations = 0;
        totalTime = 0;
    }

    public int getIterations() {
        return iterations;
    }

    public double getElapsedTime() {
        return totalTime;
    }

    public double[] getState() {
        return state;
    }
}
