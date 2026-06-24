package org.firstinspires.ftc.teamcode.utils.math.calc;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;

public class Vector2D extends Vector {
    private static Vector2D zero = new Vector2D(0, 0);

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

    public static Vector2D fromVector(Vector v) {
        return new Vector2D(v.getXComponent(), v.getYComponent());
    }

    @Override
    public Vector2D normalize() {
        return (Vector2D) super.normalize();
    }

    public double quadraticForm(Matrix m) {
        return dot(transform(m));
    }

    public static Vector2D fromPose(Pose p) {
        return new Vector2D(p.getX(), p.getY());
    }

    public Pose toPose() {
        return withHeading(0);
    }

    public Pose withHeading(double heading) {
        return new Pose(getX(), getY(), heading);
    }

    @Override
    public Vector2D plus(Vector other) {
        return new Vector2D(this.getXComponent() + other.getXComponent(),
                this.getYComponent() + other.getYComponent());
    }

    @Override
    public Vector2D times(double scalar) {
        return new Vector2D(this.getXComponent() * scalar,
                this.getYComponent() * scalar);
    }

    public static String print(Vector vector) {
        return "x: " + vector.getXComponent() + " y: " + vector.getYComponent();
    }

    public static Vector2D zero() {
        return zero;
    }
}
