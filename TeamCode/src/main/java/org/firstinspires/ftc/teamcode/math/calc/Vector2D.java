package org.firstinspires.ftc.teamcode.math.calc;
import com.pedropathing.math.Vector;

public class Vector2D extends Vector {
    public Vector2D(double x, double y) {
        super();
        setOrthogonalComponents(x, y);
    }

    public double distSquared(Vector2D other) {
        Vector offset = this.minus(other);
        return offset.dot(offset);
    }

    public double magSquared() {
        return this.dot(this);
    }

    public double getX() {
        return getXComponent();
    }

    public double getY() {
        return getYComponent();
    }

    public Vector rotate(double theta) {
        Vector vector = this.copy();
        vector.rotateVector(theta);
        return vector;
    }
}
