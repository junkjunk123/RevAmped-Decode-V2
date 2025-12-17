package org.firstinspires.ftc.teamcode.revamped.math.calc;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Objects;
import java.util.Optional;

/**
 * Time-invariant, zero-tuning extrapolator using finite differences.
 * Stores the last `order` values in a circular buffer and predicts the next value.
 */
public class Extrapolator {
    private final double[] buffer;      // circular buffer of previous values
    private int head = 0;               // index of newest value
    private final double[] coefficients;
    private final int order;
    private int size = 0;               // number of values added so far

    private static final HashMap<Integer, double[]> coefficientsCache = new HashMap<>();
    static {
        coefficientsCache.put(0, new double[]{1.0});
        coefficientsCache.put(1, new double[]{2.0, -1.0});
        coefficientsCache.put(2, new double[]{3.0, -3.0, 1.0});
        coefficientsCache.put(3, new double[]{4.0, -6.0, 4.0, -1.0});
        coefficientsCache.put(4, new double[]{5.0, -10.0, 10.0, -5.0, 1.0});
        coefficientsCache.put(5, new double[]{6.0, -15.0, 20.0, -15.0, 6.0, -1.0});
    }

    public Extrapolator(int order) {
        if (order < 1) throw new IllegalArgumentException("Order must be >= 1");
        this.order = order;
        buffer = new double[order];
        coefficients = getCoefficients(order);
    }

    /**
     * Returns the coefficients for the given order, using cache or Pascal's triangle.
     */
    public static double[] getCoefficients(int order) {
        if (coefficientsCache.containsKey(order)) {
            return coefficientsCache.get(order);
        }

        int[][] C = new int[order + 2][order + 2];
        for (int i = 0; i <= order + 1; i++) {
            C[i][0] = 1;
            for (int j = 1; j <= i; j++) {
                C[i][j] = C[i-1][j-1] + C[i-1][j];
            }
        }

        double[] coeffs = new double[order + 1];
        for (int j = 0; j <= order; j++) {
            coeffs[j] = ((j % 2 == 0) ? 1.0 : -1.0) * C[order + 1][j + 1];
        }
        coefficientsCache.put(order, coeffs);
        return coeffs;
    }

    public void update(double newValue) {
        if (size < order) size++;
        buffer[head] = newValue;
        head = (head + 1) % order; // move head forward
    }

    /**
     * Adds a new value and computes the extrapolated next value.
     */
    public Optional<Double> extrapolate() {
        if (size < 2) return Optional.empty();
        if (size < order) {
            int s = Math.min(size, 5);
            int index = (head - 1 + order) % order;
            double result = 0.0;
            double[] coeffs = coefficientsCache.get(s);
            for (int i = 0; i < s; i++) {
                result += Objects.requireNonNull(coeffs)[i] * buffer[index];
                index = (index - 1 + order) % order; // circular decrement
            }
            return Optional.of(result);
        }

        double result = 0.0;
        int index = (head - 1 + order) % order;
        for (double coeff : coefficients) {
            result += coeff * buffer[index];
            index = (index - 1 + order) % order; // circular decrement
        }
        return Optional.of(result);
    }

    /** Resets the history buffer to zero. */
    public void reset() {
        Arrays.fill(buffer, 0.0);
        head = 0;
        size = 0;
    }

    /** Returns the most recent value added. */
    public double getLastValue() {
        return buffer[(head - 1 + order) % order];
    }

    /** Returns the order of the extrapolator. */
    public int getOrder() {
        return order;
    }
}