package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Matrix;

import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeBlobCamera;
import org.firstinspires.ftc.teamcode.utils.data.Point;
import org.firstinspires.ftc.teamcode.utils.data.SimpleBlob;
import org.firstinspires.ftc.teamcode.utils.math.calc.Vector3D;

import java.util.Arrays;

public class BlobTransformer {
    public static double PHI = 120; //deg
    public static double FOCAL_LENGTH = (DecodeBlobCamera.resWidth / 2.0) / Math.tan(PHI / 2);

    public static Matrix K = new Matrix(
            new double[][]{
                    {FOCAL_LENGTH, 0, DecodeBlobCamera.resWidth / 2.0},
                    {0, FOCAL_LENGTH, DecodeBlobCamera.resHeight / 2.0},
                    {0, 0, 1}
            }
    );
    public static Matrix K_INV = K.inverse();

    private Matrix homography;

    public BlobTransformer(double theta) {
        homography = K.times(Matrix.createRotation(theta)).times(K_INV);
    }

    public SimpleBlob transform(SimpleBlob blob) {
        Point[] blobPoints = blob.points();
        Point[] transformed = Arrays.stream(blobPoints)
                .map(p -> {
                    Vector3D homogeneousCoords = new Vector3D(p.x(), p.y(), 1);
                    Vector3D transformedCoords = homogeneousCoords.transform(homography);
                    return new Point(transformedCoords.getX(), transformedCoords.getY()).times(1.0 / transformedCoords.getZ());
                })
                .toArray(Point[]::new);
        return new SimpleBlob(transformed);
    }
}
