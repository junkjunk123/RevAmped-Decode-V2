package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

/**
 * LimeLightStream is a class that implements CameraStreamSource to provide a stream of frames from a LimeLight camera.
 * It uses OpenCV to capture frames and convert them to Bitmap format.
 */
public class LimeLightStream implements CameraStreamSource {
    /**
     * VideoCapture is used to capture video frames from the LimeLight camera.
     */
    VideoCapture capture;

    /**
     * Constructor for LimeLightStream.
     * Initializes the VideoCapture object with the URL of the LimeLight camera stream.
     */
    public LimeLightStream(){
        capture = new VideoCapture("http://172.29.0.1:5800/stream.mjpg");
    }

    /**
     * Checks if the camera is open and ready to capture frames.
     * @return true if the camera is open, false otherwise
     */
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        Mat frame = new Mat();
        capture.read(frame);
        continuation.dispatch((ContinuationResult<Consumer<Bitmap>>) consumer -> consumer.accept(convertMatToBitMap(frame)));
    }

    /**
     * Converts a Mat object to a Bitmap.
     * @param input the Mat object to be converted
     * @return the converted Bitmap
     */
    public Bitmap convertMatToBitMap(Mat input){
        Bitmap bmp;
        Imgproc.cvtColor(input, input,Imgproc.COLOR_BGR2RGB);
        bmp = Bitmap.createBitmap(input.cols(), input.rows(),Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(input,bmp);
        return bmp;
    }
}