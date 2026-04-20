package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeBlobCamera;
import org.firstinspires.ftc.teamcode.utils.Globals;

@TeleOp
public class BlobCameraTest extends OpMode {
    private DecodeBlobCamera camera;
    @Override
    public void init() {
        Globals.init(telemetry);
        camera = new DecodeBlobCamera(hardwareMap);
    }

    @Override
    public void loop() {
        camera.update();
    }
}
