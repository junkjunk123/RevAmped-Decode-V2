package org.firstinspires.ftc.teamcode.utils.data;

import org.opencv.core.RotatedRect;

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

    public static Point of(org.opencv.core.Point p) {
        return new Point(p.x, p.y);
    }

    public static Point[] getPoints(RotatedRect rect) {
        org.opencv.core.Point[] pts = new org.opencv.core.Point[4];
        rect.points(pts);
        return new Point[] {of(pts[0]), of(pts[1]), of(pts[2]), of(pts[3])};
    }
}
