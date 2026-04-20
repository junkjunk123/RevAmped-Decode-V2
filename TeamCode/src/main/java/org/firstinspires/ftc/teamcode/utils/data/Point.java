package org.firstinspires.ftc.teamcode.utils.data;

public record Point(double x, double y) {
    public double dist(Point other) {
        return Math.hypot(x - other.x, y - other.y);
    }
}
