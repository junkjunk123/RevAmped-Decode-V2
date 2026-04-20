package org.firstinspires.ftc.teamcode.mechanisms.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.hardware.BlobProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class DecodeBlobCamera {
    private VisionPortal portal;
    private BlobProcessor purpleBlobProcessor;
    private BlobProcessor greenBlobProcessor;
    public static int resWidth = 1920;
    public static int resHeight = 1080;
    public DecodeBlobCamera(HardwareMap hardwareMap){
        purpleBlobProcessor = new BlobProcessor(ColorRange.ARTIFACT_PURPLE);
        purpleBlobProcessor.addPreFilter(new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,50,999999999));
        greenBlobProcessor = new BlobProcessor(ColorRange.ARTIFACT_GREEN);
        greenBlobProcessor.addPreFilter(new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,50,999999999));
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"blobcam"))
                .setCameraResolution(new Size(resWidth,resHeight))
                .addProcessor(purpleBlobProcessor.getProcessor())
                .addProcessor(greenBlobProcessor.getProcessor())
                .build();
    }

    public void update(){
        purpleBlobProcessor.update();
        greenBlobProcessor.update();
    }

    public List<ColorBlobLocatorProcessor.Blob> getPurpleBlobs(){
        return purpleBlobProcessor.getBlobs();
    }

    public List<ColorBlobLocatorProcessor.Blob> getGreenBlobs(){
        return greenBlobProcessor.getBlobs();
    }

    public List<ColorBlobLocatorProcessor.Blob> getAllBlobs(){
        return Stream.concat(getPurpleBlobs().stream(), getGreenBlobs().stream())
                .collect(Collectors.toList());
    }
}
