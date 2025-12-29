package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.math.Matrix;
import com.pedropathing.math.MatrixUtil;

public class MathUtil {
    public static Matrix invert(Matrix m) {
        if (m.getRows() != m.getColumns())
            throw new IllegalStateException("Matrix must be square");

        Matrix I = MatrixUtil.eye(m.getRows());
        Matrix[] r = Matrix.rref(m, I);

        if (!r[1].equals(I)) throw new IllegalArgumentException("matrix not invertible");
        return r[1];
    }
}
