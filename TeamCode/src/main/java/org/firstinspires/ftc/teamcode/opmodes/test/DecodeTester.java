package org.firstinspires.ftc.teamcode.opmodes.test;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DecodeTester extends SelectableOpMode {
    public DecodeTester() {
        super("Select a Testing OpMode.", s -> {
            s.folder("Limelight", l -> {
                l.add("ObeliskDetectionTest", ObeliskDetectionTest::new);
            });
            s.folder("Calibrator", l -> {
                l.add("ServoCalibrateTest", ServoCalibrateTest::new);
            });
            s.folder("Turntable", l -> {
                l.add("TableEncoderTest", TableEncoderTest::new);
            });
            s.folder("PedroTest", l -> {
                l.add("PedroDrivePChecker", PedroDrivePChecker::new);
                l.add("PedroPIDChecker", PedroPIDChecker::new);
            });
        });
    }
}
