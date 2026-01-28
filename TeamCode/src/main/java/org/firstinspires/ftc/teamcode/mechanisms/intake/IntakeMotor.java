package org.firstinspires.ftc.teamcode.mechanisms.intake;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;

public class IntakeMotor extends HwMotor {
    public static float INTAKE;
    public static float OUTTAKE;
    public static float INTAKE_SLOW;
    public static float SHOOTING;
    public static float OUTTAKE_SLOW;
    public static float STOPPED;

    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        INTAKE_SLOW,
        SHOOTING,
        OUTTAKE_SLOW,
        STOPPED
    }
    private IntakeState state = IntakeState.STOPPED;

    public IntakeMotor(HardwareMap hardwareMap) {
        super(hardwareMap, false, "intake");
        setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void intake() {
        setPower(INTAKE);
        state = IntakeState.INTAKE;
    }

    public void outtake() {
        setPower(OUTTAKE);
        state = IntakeState.OUTTAKE;
    }

    public void intakeSlow() {
        setPower(INTAKE_SLOW);
        state = IntakeState.INTAKE_SLOW;
    }

    public void shooting() {
        setPower(SHOOTING);
        state = IntakeState.SHOOTING;
    }

    public void outtakeSlow() {
        setPower(OUTTAKE_SLOW);
        state = IntakeState.OUTTAKE_SLOW;
    }

    public void stop() {
        setPower(STOPPED);
        state = IntakeState.STOPPED;
    }

    public String getState() {
        return state.name();
    }

    public boolean atState(IntakeState state) {
        return state == this.state;
    }
}
