package org.firstinspires.ftc.teamcode.utils.vision;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.utils.data.Point;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.Point3;
import org.opencv.core.RotatedRect;

import java.util.Arrays;
import java.util.Iterator;

public class SimpleBlob implements Iterable<Point> {
    public final Point one;
    public final Point two;
    public final Point three;
    public final Point four;

    public final Point center;
    private double area = -1;

    public SimpleBlob(Point center, Point one, Point two, Point three, Point four) {
        this.one = one;
        this.two = two;
        this.three = three;
        this.four = four;
        this.center = center;
    }

    public SimpleBlob(ColorBlobLocatorProcessor.Blob blob) {
        this(Point.of(blob.getBoxFit().center), Point.getPoints(blob.getBoxFit()));
        area = blob.getContourArea();
    }

    public SimpleBlob(Point center, Point[] points) {
        this(center, points[1], points[2], points[3], points[4]);
        if (points.length != 5)
            throw new IllegalArgumentException("Blob is a quadrilateral please cause I'm not coding more complex ones");
    }

    public double area() {
        if (area == -1) {
            double sumOne = one.x() * two.y() + two.x() * three.y() + three.x() * four.y() + four.x() * one.y();
            double sumTwo = one.y() * two.x() + two.y() * three.x() + three.y() * four.x() + four.y() * one.x();
            area = Math.abs(sumOne - sumTwo) / 2.0;
        }

        return area;
    }

    @NonNull
    @Override
    public Iterator<Point> iterator() {
        return new Iterator<>() {
            int i = 0;

            @Override
            public boolean hasNext() {
                return i <= 3;
            }

            @Override
            public Point next() {
                i++;
                return get(i);
            }
        };
    }

    public Point get(int i) {
        return switch (i) {
            case 1 -> one;
            case 2 -> two;
            case 3 -> three;
            default -> four;
        };
    }

    public Point[] points() {
        return new Point[] {one, two, three, four};
    }
}
