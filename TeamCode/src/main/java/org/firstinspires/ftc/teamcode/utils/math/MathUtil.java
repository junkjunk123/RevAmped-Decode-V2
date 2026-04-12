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
}
