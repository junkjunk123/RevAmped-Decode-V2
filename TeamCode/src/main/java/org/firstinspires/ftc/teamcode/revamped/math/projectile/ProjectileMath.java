package org.firstinspires.ftc.teamcode.revamped.math.projectile;

import java.util.Arrays;

public class ProjectileMath {
    private static final double g = -386.0885; // gravity in in/s^2

    /**
     * Solve for possible launch angles theta (radians).
     * Only returns positive angles (0 < theta < pi/2).
     *
     * @param v initial speed
     * @param h initial height
     * @param x horizontal distance to target
     * @param y target height
     * @return array of positive solutions (length 0, 1, or 2) in radians
     */
    public static double solveThetaWithoutDrag(double v, double h, double x, double y) {
        if (x == 0) {
            return -1; // avoid division by zero
        }

        // coefficients
        double A = (g * x * x) / (2 * v * v);
        double C = h - y + A;

        // discriminant
        double disc = x * x - 4 * A * C;
        if (disc < 0) {
            return -1; // no real solution
        }

        double sqrtDisc = Math.sqrt(disc);

        double T1 = (-x + sqrtDisc) / (2 * A);
        double T2 = (-x - sqrtDisc) / (2 * A);

        // convert to angles and filter only positive
        java.util.List<Double> solutions = new java.util.ArrayList<>();

        if (disc == 0) {
            double theta = Math.atan(T1);
            if (theta > 0) solutions.add(theta);
        } else {
            double theta1 = Math.atan(T1);
            double theta2 = Math.atan(T2);

            if (theta1 > 0) solutions.add(theta1);
            if (theta2 > 0) solutions.add(theta2);
        }

        // convert list to array
        double[] result = new double[solutions.size()];
        for (int i = 0; i < solutions.size(); i++) {
            result[i] = solutions.get(i);
        }

        return Arrays.stream(result).min().orElse(-1);
    }
}
