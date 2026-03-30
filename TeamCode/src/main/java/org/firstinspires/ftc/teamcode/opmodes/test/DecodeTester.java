package org.firstinspires.ftc.teamcode.opmodes.test;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DecodeTester extends SelectableOpMode {
    public DecodeTester() {
        super("Select a Testing OpMode.", s -> {
            s.folder("Vision", l -> {
                l.add("ObeliskDetectionTest", ObeliskDetectionTest::new);
                l.add("HuskyLensTest", HuskyLensTest::new);
            });
            s.folder("Calibrator", l -> {
                l.add("FlywheelTuner", FlywheelTuner::new);
                l.add("TurretPIDTuner", TurretPIDTuner::new);
            });
            s.folder("HardwareTest", l -> {
                l.add("DriveMotorFrictionTest", DriveMotorFrictionTest::new);
                l.add("TableEncoderTest", TableEncoderTest::new);
                l.add("TurretEncoderTest", TurretEncoderTest::new);
            });
            s.folder("PedroTest", l -> {
                l.add("PedroDrivePChecker", PedroDrivePChecker::new);
                l.add("PedroPIDChecker", PedroPIDChecker::new);
            });
            s.folder("Pre-Match Test", l -> {
                l.add("AutomatedTest", AutomatedTest::new);
            });
            s.folder("Software Test", l -> {
                l.add("SortTest", SortTest::new);
                l.add("UltrasonicEKFTest", UltrasonicEKFTest::new);
                l.add("TurretDataTransferTest", TurretSim::new);
            });
        });
    }

    @Override
    protected void onSelect() {
        telemetry.addLine("Finished selecting OpMode.");
    }
}
