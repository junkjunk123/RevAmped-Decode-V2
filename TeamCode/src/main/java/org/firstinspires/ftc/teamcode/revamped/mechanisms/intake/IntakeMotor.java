package org.firstinspires.ftc.teamcode.revamped.mechanisms.intake;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.revamped.utils.hardware.HwMotor;

public class IntakeMotor extends HwMotor {
    public IntakeMotor(HardwareMap hardwareMap) {
        super(hardwareMap, "intakeMotor");
    }

    public void intake() {
        setPower(1.0);
    }

    public void outtake() {
        setPower(-1.0);
    }

    public void intakeSlow() {
        setPower(0.5);
    }

    public void shooting() {
        setPower(0.4);
    }

    public void slowOuttake() {
        setPower(-0.5);
    }

    public void stop() {
        setPower(0.0);
    }
}
