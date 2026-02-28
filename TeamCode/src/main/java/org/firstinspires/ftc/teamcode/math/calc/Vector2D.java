package org.firstinspires.ftc.teamcode.math.calc;

import com.pedropathing.geometry.Pose;
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

    public static Vector2D fromVector(Vector v) {
        return new Vector2D(v.getXComponent(), v.getYComponent());
    }

    public static Vector2D fromPose(Pose p) {
        return new Vector2D(p.getX(), p.getY());
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
}
