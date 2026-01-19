package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestTeleop", group = "TeleOp")
public class testTeleop extends OpMode {
    private testDrivetrain drivetrain;

    @Override
    public void init() {
        // Initialize the drivetrain
        drivetrain = new testDrivetrain(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get controller inputs
        double x = gamepad1.left_stick_x;   // Strafe
        double y = -gamepad1.left_stick_y;  // Forward/back (negative because Y is inverted)
        double rx = gamepad1.right_stick_x; // Rotation

        // Drive the robot
        drivetrain.arcadeDrive(x, y, rx);

        // Show telemetry
        telemetry.addData("X (Strafe)", x);
        telemetry.addData("Y (Forward)", y);
        telemetry.addData("RX (Rotate)", rx);
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
    }
}