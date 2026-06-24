package org.firstinspires.ftc.teamcode.utils.math;

import org.firstinspires.ftc.teamcode.utils.data.Pair;

public class LambertApproximator {
    // Parameters of your system
    private final double a;
    private final double b;
    private final double v0;

    private double v0actual;

    private final double WX_0;
    private double WX_new;

    public static double DEFAULT_DT = 0.5;
    private double defaultV_Factor;
    private double defaultD_factor;

    public LambertApproximator(double a, double b, double v0) {
        this.a = a;
        this.b = b;
        this.v0 = v0;
        v0actual = v0;
        WX_0 = lambertW(computeX(0));
        setDefaultDt(DEFAULT_DT);
    }

    /**
     * Evaluates the inner argument x(t) = (2a/b) * A * exp((2a/b)*A - t/b)
     */
    private double computeX(double t) {
        double leadingTerm = (2.0 * a / b) * v0;
        return leadingTerm * Math.exp(leadingTerm - (t / b));
    }

    /**
     * Computes the Lambert W function (principal branch W_0) using Halley's Method.
     * This converges cubically and handles small/large values gracefully.
     */
    public double lambertW(double x) {
        if (x < 0) {
            // Adjust or throw error based on your domain constraints.
            // Principal branch W0(x) is only real-valued for x >= -1/e.
            if (x < -1.0 / Math.E) return Double.NaN;
        }

        // Initial guess (good for x > 0)
        double w = x > 1.0 ? Math.log(x) - Math.log(Math.log(x)) : x;
        if (w < 0) w = 0.0; // clamp for safety near 0

        // 2-3 iterations of Halley's method usually hit machine precision
        for (int i = 0; i < 3; i++) {
            double ew = Math.exp(w);
            double f = w * ew - x;
            double num = 2.0 * ew * (1.0 + w);

            double delta = f / (num - (f / (w + 1.0)));
            w -= delta;

            if (Math.abs(delta) < 1e-15) break;
        }
        return w;
    }

    public void setV0actual(double v0actual) {
        this.v0actual = v0actual;
    }

    /**
     * Computes f(t) = W(x(t))
     */
    public double computeVelocity(double t) {
        double x = computeX(t);
        return lambertW(x) * b / 2 / a;
    }

    /**
     * Computes the exact integral of f(t) from t to t + dt
     * Formula: b * [ (W_t^2 / 2 + W_t) - (W_tdt^2 / 2 + W_tdt) ]
     */
    public double computeDisplacement(double t, double dt) {
        double w_t = lambertW(computeX(t));
        double w_tdt = lambertW(computeX(t + dt));

        double upperEval = (w_tdt * w_tdt / 2.0) + w_tdt;
        double lowerEval = (w_t * w_t / 2.0) + w_t;

        return b * b * (lowerEval - upperEval) / 2 / a;
    }

    public Pair<Double, Double> compute(double t, double dt) {
        double v_now = lambertW(computeX(t));
        double v_next = lambertW(computeX(t + dt));
        double ratio = v0actual / v0 * b / 2 / a;
        double upperEval = (v_next * v_next / 2.0) + v_next;
        double lowerEval = (v_now * v_now / 2.0) + v_now;
        double integral = b * (lowerEval - upperEval);
        return new Pair<>(integral * ratio, v_next * ratio);
    }

    public Pair<Double, Double> compute(double dt) {
        double v_next = lambertW(computeX(dt));
        double ratio = v0actual / v0 * b / 2 / a;
        double upperEval = (v_next * v_next / 2.0) + v_next;
        double lowerEval = (WX_0 * WX_0 / 2.0) + WX_0;
        double integral = b * (lowerEval - upperEval);
        return new Pair<>(integral * ratio, v_next * ratio);
    }

    public Pair<Double, Double> compute() {
        return new Pair<>(defaultD_factor * v0actual, defaultV_Factor * v0actual);
    }

    public void setDefaultDt(double defaultDt) {
        DEFAULT_DT = defaultDt;
        WX_new = lambertW(computeX(DEFAULT_DT));
        double ratio = 1 / v0 * b / 2 / a;
        double upperEval = (WX_new * WX_new / 2.0) + WX_new;
        double lowerEval = (WX_0 * WX_0 / 2.0) + WX_0;
        double integral = b * (lowerEval - upperEval);
        defaultV_Factor = WX_new * ratio;
        defaultD_factor = integral * ratio;
    }
}