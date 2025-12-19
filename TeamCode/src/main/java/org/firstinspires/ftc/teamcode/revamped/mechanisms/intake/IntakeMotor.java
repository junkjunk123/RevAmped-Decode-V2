package org.firstinspires.ftc.teamcode.revamped.mechanisms.intake;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.revamped.utils.hardware.HwMotor;

public class IntakeMotor extends HwMotor {
    public static float INTAKE;
    public static float OUTTAKE;
    public static float INTAKE_SLOW;
    public static float SHOOTING;
    public static float OUTTAKE_SLOW;
    public static float STOPPED;

    public IntakeMotor(HardwareMap hardwareMap) {
        super(hardwareMap, "intakeMotor");
    }

    public void intake() {
        setPower(INTAKE);
    }

    public void outtake() {
        setPower(OUTTAKE);
    }

    public void intakeSlow() {
        setPower(INTAKE_SLOW);
    }

    public void shooting() {
        setPower(SHOOTING);
    }

    public void outtakeSlow() {
        setPower(OUTTAKE_SLOW);
    }

    public void stop() {
        setPower(STOPPED);
    }
}
