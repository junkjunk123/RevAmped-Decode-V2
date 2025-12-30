package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.LimeLightStream;

@TeleOp(name = "LimelightDashboard", group = "Test")
public class LimelightTest extends OpMode {

    @Override
    public void init() {
        initialize();
    }

    @Override
    public void start() {
        LimeLightStream stream = new LimeLightStream();
        FtcDashboard.getInstance().startCameraStream(stream,30);
    }

    @Override
    public void loop() {}

    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }
}