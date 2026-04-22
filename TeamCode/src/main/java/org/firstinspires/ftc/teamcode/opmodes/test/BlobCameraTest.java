package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeBlobCamera;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.hardware.BlobProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

@TeleOp
public class BlobCameraTest extends OpMode {
    private DecodeBlobCamera camera;
    @Override
    public void init() {
        Globals.init(telemetry);
        BlobProcessor.debug = true;
        camera = new DecodeBlobCamera(hardwareMap);
    }

    @Override
    public void start() {
        camera.start();
    }

    @Override
    public void loop() {
        camera.update();
        telemetry.addData("blobs #",camera.getAllBlobs().size());
        for(ColorBlobLocatorProcessor.Blob b : camera.getAllBlobs()){
            telemetry.addData("center",b.getBoxFit().center);
            telemetry.addData("centerx",b.getBoxFit().center.x);
            telemetry.addData("centery",b.getBoxFit().center.y);
        }
        telemetry.update();
    }
}
