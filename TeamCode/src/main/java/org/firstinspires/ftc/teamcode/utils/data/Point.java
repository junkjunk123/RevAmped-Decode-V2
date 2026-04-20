package org.firstinspires.ftc.teamcode.utils.data;

public record Point(double x, double y) {
    public double dist(Point other) {
        return Math.hypot(x - other.x, y - other.y);
    }

    public Point plus(Point other) {
        return new Point(x + other.x, y + other.y);
    }

    public Point times(double scalar) {
        return new Point(x * scalar, y * scalar);
    }
}
