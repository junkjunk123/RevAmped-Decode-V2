package org.firstinspires.ftc.teamcode.utils.vision;

import com.pedropathing.math.Matrix;

import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeBlobCamera;
import org.firstinspires.ftc.teamcode.utils.data.Point;
import org.firstinspires.ftc.teamcode.utils.math.calc.Vector3D;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.TreeSet;

public class BlobTransformer {
    public static double PHI;
    public static double FOCAL_LENGTH;

    public static double X_1 = 594; //near-middle
    public static double X_2 = 920; //middle-far
    public static double X_MIN = 260;

    public static double Y_MAX = 480;

    public static Matrix K;
    public static Matrix K_INV;
    private final Matrix homography;

    public BlobTransformer(double theta) {
        homography = K.times(Matrix.createRotation(theta)).times(K_INV);
    }

    public SimpleBlob transform(SimpleBlob blob) {
        Point[] blobPoints = blob.points();
        Point[] transformed = Arrays.stream(blobPoints)
                .map(this::transform)
                .toArray(Point[]::new);
        Point newCenter = transform(blob.center);
        return new SimpleBlob(newCenter, transformed);
    }

    private Point transform(Point p) {
        Vector3D homogeneousCoords = new Vector3D(p.x(), p.y(), 1);
        Vector3D transformedCoords = homogeneousCoords.transform(homography);
        return new Point(transformedCoords.getX(), transformedCoords.getY()).times(1.0 / transformedCoords.getZ());
    }

    public static int computeIntakeRegion(List<ColorBlobLocatorProcessor.Blob> blobs, double curHeading) {
        if (blobs.isEmpty()) return -1;
        BlobTransformer transformer = new BlobTransformer(curHeading);
        SimpleBlob[] blobArray = blobs.stream()
                .map(b -> transformer.transform(new SimpleBlob(b)))
                .filter(b -> b.center.y() < Y_MAX && b.center.x() > X_MIN)
                .toArray(SimpleBlob[]::new);
        return transformer.computeIntakeRegion(blobArray);
    }

    public int computeIntakeRegion(SimpleBlob[] blobArray) {
        if (blobArray.length == 0) return -1;

        TreeSet<Double> xSet = new TreeSet<>();
        xSet.add(X_1);
        xSet.add(X_2);

        for (SimpleBlob b : blobArray)
            for (Point p : b.points())
                xSet.add(p.x());

        Double[] xVals = xSet.toArray(new Double[0]);
        double[] area = new double[3];

        for (int i = 0; i < xVals.length - 1; i++) {
            double xL = xVals[i];
            double xR = xVals[i + 1];
            if (xR <= xL) continue; //robot should just die atp

            double xMid = 0.5*(xL + xR);

            List<double[]> intervals = new ArrayList<>();

            for (SimpleBlob b : blobArray) {
                int count = 0;
                double yMin = Double.POSITIVE_INFINITY;
                double yMax = Double.NEGATIVE_INFINITY;

                for (int j = 0; j < 4; j++) {
                    Point p1 = b.get(j);
                    Point p2 = b.get((j + 1) % 4);

                    if ((p1.x() >= xMid && p2.x() <= xMid) || (p1.x() <= xMid && p2.x() >= xMid)) {
                        if (p1.x() == p2.x()) continue; //camera is prolly having an aneurysm atp
                        double t = (xMid - p1.x()) / (p2.x() - p1.x());

                        if (t >= 0 && t <= 1) {
                            double y = p1.y() + t * (p2.y() - p1.y());

                            if (y < yMin) yMin = y;
                            if (y > yMax) yMax = y;
                            count++;
                        }
                    }
                }

                if (count > 2) {
                    intervals.add(new double[] {yMin, yMax});
                }
            }

            if (intervals.isEmpty()) continue;
            intervals.sort(Comparator.comparingDouble(a -> a[0]));

            double totalY = 0;
            double curL = intervals.get(0)[0];
            double curR = intervals.get(0)[1];

            for (int j = 1; j < intervals.size(); j++) {
                double[] curInterval = intervals.get(j);

                if (curInterval[0] <= curR)
                    curR = Math.max(curR, curInterval[1]);
                else {
                    totalY += curR - curL;
                    curL = curInterval[0];
                    curR = curInterval[1];
                }
            }

            totalY += curR - curL;
            double dA = totalY * (xR - xL);

            int region = getRegion(xMid);
            area[region] += dA;
        }

        int bestRegion = 0;
        if (area[1] > area[bestRegion]) bestRegion = 1;
        if (area[2] > area[bestRegion]) bestRegion = 2;

        return bestRegion;
    }

    private int getRegion(double xMid) {
        if (xMid < X_1) return 0;
        else if (xMid < X_2) return 1;
        else return 2;
    }
}
