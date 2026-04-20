package org.firstinspires.ftc.teamcode.utils.data;

public class SimpleBlob {
    public Point one;
    public Point two;
    public Point three;
    public Point four;

    private double area = -1;

    public double area() {
        if (area == -1) {
            double sumOne = one.x() * two.y() + two.x() * three.y() + three.x() * four.y() + four.x() * one.y();
            double sumTwo = one.y() * two.x() + two.y() * three.x() + three.y() * four.x() + four.y() * one.x();
            area = Math.abs(sumOne - sumTwo) / 2.0;
        }

        return area;
    }
}
