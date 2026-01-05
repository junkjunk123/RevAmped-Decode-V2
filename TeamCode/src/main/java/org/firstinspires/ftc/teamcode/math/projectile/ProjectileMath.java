package org.firstinspires.ftc.teamcode.math.projectile;

import org.firstinspires.ftc.teamcode.math.calc.Vector2D;

import java.util.Arrays;

public class ProjectileMath {
    private static final double g = 386.0885; // gravity in in/s^2

    /**
     * Solve for possible launch angles theta (radians).
     * Only returns positive angles (0 < theta < pi/2).
     *
     * @param distances a vector whose x-component is the cartesian xy distance to the target and whose y-component is the vertical distance to the target
     * @param v0 the launch velocity of the artifact
     * @return array of positive solutions (length 0, 1, or 2) in radians
     */
    public static double solveTheta(Vector2D distances, double v0, double currentHoodRad) {
        if (distances.getX() == 0) {
            return -1; // avoid division by zero
        }

        // coefficients
        double A = (-g * distances.getX() * distances.getX()) / (2 * v0 * v0);
        double C = -distances.getY() + A;

        // discriminant
        double disc = distances.getX() * distances.getX() - 4 * A * C;
        if (disc < 0) {
            return -1; // no real solution
        }

        double sqrtDisc = Math.sqrt(disc);

        double T1 = (-distances.getX() + sqrtDisc) / (2 * A);
        double T2 = (-distances.getX() - sqrtDisc) / (2 * A);

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

        return Arrays.stream(result).min().orElse(currentHoodRad);
    }
}
