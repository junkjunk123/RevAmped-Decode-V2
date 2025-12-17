package org.firstinspires.ftc.teamcode.revamped.mechanisms.intake;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.HwDigitalDevice;

public class IntakeDistance extends HwDigitalDevice {
    public IntakeDistance(HardwareMap hardwareMap) {
        super(hardwareMap, "intakeDistance");
    }

    public boolean hasArtifact() {
        return state();
    }
}
