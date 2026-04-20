package org.firstinspires.ftc.teamcode.utils.math.calc;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Matrix;

/**
 * A class representing a three-dimensional vector with additional operations
 */
public class Vector3D {
    double[] values = new double[3];

    /**
     * Constructs a Vector3D with the specified x, y, and z coordinates.
     *
     * @param x the x coordinate
     * @param y the y coordinate
     * @param z the z coordinate
     */
    public Vector3D(double x, double y, double z) {
        values[0] = x;
        values[1] = y;
        values[2] = z;
    }

    /**
     * Returns the cross product of this vector and another vector.
     * @param other the other vector to cross with
     * @return a new Vector3D representing the cross product
     */
    public Vector3D crossProduct(Vector3D other) {
        double x = get(1) * other.get(2) - get(2) * other.get(1);
        double y = get(2) * other.get(0) - get(0) * other.get(2);
        double z = get(0) * other.get(1) - get(1) * other.get(0);
        return new Vector3D(x, y, z);
    }

    /**
     * Calculates the volume of the parallelepiped formed by this vector and two other vectors.
     * @param v2 the second vector
     * @param v3 the third vector
     * @return the volume of the parallelepiped
     */
    public double volume(Vector3D v2, Vector3D v3) {
        Vector3D cross = crossProduct(v2);
        return Math.abs(cross.dotProduct(v3));
    }

    /**
     * Converts this vector to spherical coordinates.
     * @return an array containing the radius, polar angle (theta), and azimuthal angle (phi)
     */
    public double[] toSpherical() {
        double x = get(0);
        double y = get(1);
        double z = get(2);

        double r = magnitude(); // Radius
        double theta = Math.toDegrees(Math.acos(z / r)); // Polar angle
        double phi = Math.toDegrees(Math.atan2(y, x)); // Azimuthal angle

        return new double[]{r, theta, phi};
    }

    /**
     * Creates a Vector3D from spherical coordinates.
     * @param r the radius
     * @param theta the polar angle in degrees
     * @param phi the azimuthal angle in degrees
     * @return a new Vector3D representing the point in Cartesian coordinates
     */
    public static Vector3D fromSpherical(double r, double theta, double phi) {
        double thetaRadians = Math.toRadians(theta); // Convert polar angle to radians
        double phiRadians = Math.toRadians(phi);     // Convert azimuthal angle to radians

        double x = r * Math.sin(thetaRadians) * Math.cos(phiRadians);
        double y = r * Math.sin(thetaRadians) * Math.sin(phiRadians);
        double z = r * Math.cos(thetaRadians);

        return new Vector3D(x, y, z);
    }

    public double get(int i) {
        return values[i];
    }

    public double getX() {
        return values[0];
    }

    public double getY() {
        return values[1];
    }

    public double getZ() {
        return values[2];
    }

    public double dotProduct(Vector3D other) {
        return getX() * other.getX() + getY() * other.getY() + getZ() * other.getZ();
    }

    public double magnitudeSq() {
        return this.dotProduct(this);
    }

    public double magnitude() {
        return Math.sqrt(magnitudeSq());
    }

    public static Vector3D get3DPosition(Pose pose, double height) {
        return new Vector3D(pose.getX(), pose.getY(), height);
    }

    public double[] getValues() {
        return values;
    }

    public Vector3D copy() {
        return new Vector3D(getX(), getY(), getZ());
    }

    public Vector3D transform(Matrix matrix) {
        if (matrix.getRows() != 3 || matrix.getColumns() != 3) {
            throw new IllegalArgumentException("Matrix must be 3x3 for Vector3D transformation.");
        }

        double[] result = new double[3];
        for (int i = 0; i < 3; i++) {
            result[i] = 0;
            for (int j = 0; j < 3; j++) {
                result[i] += matrix.get(i, j) * this.get(j);
            }
        }

        return new Vector3D(result[0], result[1], result[2]);
    }

    public double quadraticForm(Matrix matrix) {
        if (matrix.getRows() != 3 || matrix.getColumns() != 3) {
            throw new IllegalArgumentException("Matrix must be 3x3 for Vector3D quadratic form.");
        }

        double result = 0;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result += this.get(i) * matrix.get(i, j) * this.get(j);
            }
        }

        return result;
    }

    public Vector3D add(Vector3D other) {
        return new Vector3D(this.getX() + other.getX(), this.getY() + other.getY(), this.getZ() + other.getZ());
    }

    public Vector3D subtract(Vector3D other) {
        return new Vector3D(this.getX() - other.getX(), this.getY() - other.getY(), this.getZ() - other.getZ());
    }

    public Vector3D scale(double scalar) {
        return new Vector3D(this.getX() * scalar, this.getY() * scalar, this.getZ() * scalar);
    }

    public Vector3D normalize() {
        double mag = this.magnitude();
        if (mag == 0) {
            throw new ArithmeticException("Cannot normalize a zero vector.");
        }
        return this.scale(1.0 / mag);
    }

    public Vector3D plus(Vector3D other) {
        return this.add(other);
    }

    public Vector3D minus(Vector3D other) {
        return this.subtract(other);
    }

    public Vector3D times(double scalar) {
        return this.scale(scalar);
    }

    public Vector3D dividedBy(double scalar) {
        if (scalar == 0) {
            throw new ArithmeticException("Cannot divide by zero.");
        }
        return this.scale(1.0 / scalar);
    }

    @NonNull
    @Override
    public String toString() {
        return "Vector3D{" +
                "x=" + getX() +
                ", y=" + getY() +
                ", z=" + getZ() +
                '}';
    }

    public double distanceTo(Vector3D other) {
        return this.subtract(other).magnitude();
    }

    public Pose toPose() {
        return new Pose(getX(), getY(), getZ());
    }

    public Matrix tensorProduct(Vector3D other) {
        Matrix result = new Matrix(3, 3);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result.set(i, j, this.get(i) * other.get(j));
            }
        }
        return result;
    }

    public Matrix outerProduct(Vector3D other) {
        return this.tensorProduct(other);
    }
}