package org.firstinspires.ftc.teamcode.utils.math;

import com.pedropathing.math.Matrix;

public class MathUtil {
    public static Matrix rotMatrix(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double[][] vals = new double[][] {
                {cos, -sin},
                {-sin, cos}
        };
        return new Matrix(vals);
    }

    /**
     * Normalizes an angle in radians to the range [-π, π).
     *
     * @param angleRadians The angle in radians to normalize.
     * @return The normalized angle in radians, in the range [-π, π).
     */
    public static double normalizeAnglePi(double angleRadians) {
        return Math.atan2(Math.sin(angleRadians), Math.cos(angleRadians));
    }
}
