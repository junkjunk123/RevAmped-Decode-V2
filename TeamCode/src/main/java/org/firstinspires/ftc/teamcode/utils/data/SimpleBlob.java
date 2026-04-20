package org.firstinspires.ftc.teamcode.utils.data;

import androidx.annotation.NonNull;

import java.util.Iterator;

public class SimpleBlob implements Iterable<Point> {
    public final Point one;
    public final Point two;
    public final Point three;
    public final Point four;

    private double area = -1;

    public SimpleBlob(Point one, Point two, Point three, Point four) {
        this.one = one;
        this.two = two;
        this.three = three;
        this.four = four;
    }

    public SimpleBlob(Point[] points) {
        this(points[0], points[1], points[2], points[3]);
        if (points.length != 4) throw new IllegalArgumentException("Blob is a quadrilateral please cause I'm not coding more complex ones");
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
                return switch (i) {
                    case 1 -> one;
                    case 2 -> two;
                    case 3 -> three;
                    default -> four;
                };
            }
        };
    }

    public Point[] points() {
        return new Point[] {one, two, three, four};
    }
}
