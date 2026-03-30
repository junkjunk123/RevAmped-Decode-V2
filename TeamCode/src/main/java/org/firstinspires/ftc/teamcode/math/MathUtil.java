package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.math.calc.Vector2D;

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

    public static Vector mean(Vector[] vectors) {
        double x = 0, y = 0;
        for (Vector vector : vectors) {
            x += vector.getXComponent();
            y += vector.getYComponent();
        }
        x /= vectors.length;
        y /= vectors.length;
        return new Vector2D(x, y);
    }

    public static Vector geometricMedian(Vector[] vectors, int iterations) {
        //Weiszfeld Solver
        Vector guess = mean(vectors);

        for (int i = 0; i < iterations; i++) {
            Vector num = new Vector();
            double denom = 0;

            for (Vector v : vectors) {
                double dist = guess.minus(v).getMagnitude() + 1e-6;
                double omega = 1/dist;
                num = num.plus(v.times(omega));
                denom += omega;
            }

            guess = num.times(1 / denom);
        }

        return guess;
    }
}
