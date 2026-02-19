package org.firstinspires.ftc.teamcode.mechanisms.intake;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;
import org.firstinspires.ftc.teamcode.utils.logging.DecodeLogger;

public class IntakeMotor extends HwMotor {
    public static float INTAKE;
    public static float OUTTAKE;
    public static float INTAKE_SLOW;
    public static float SHOOTING;
    public static float OUTTAKE_SLOW;
    public static float STOPPED;
    public static float INTAKE_GATE;
    public static float INTAKE_PRELOADS;

    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        INTAKE_SLOW,
        SHOOTING,
        OUTTAKE_SLOW,
        GATE,
        PRELOADS,
        STOPPED
    }
    private IntakeState state = IntakeState.STOPPED;

    public IntakeMotor(HardwareMap hardwareMap) {
        super(hardwareMap, "intake");
        setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void intake() {
        setState(IntakeState.INTAKE, INTAKE);
    }

    public void outtake() {
        setState(IntakeState.OUTTAKE, OUTTAKE);
    }

    public void intakeSlow() {
        setState(IntakeState.INTAKE_SLOW, INTAKE_SLOW);
    }

    public void shooting() {
        setState(IntakeState.SHOOTING, SHOOTING);
    }

    public void outtakeSlow() {
        setState(IntakeState.OUTTAKE_SLOW, OUTTAKE_SLOW);
    }

    public void stop() {
        setState(IntakeState.STOPPED, STOPPED);
    }

    public void intakeGate() {
        setState(IntakeState.GATE, INTAKE_GATE);
    }

    public void intakePreloads() {
        setState(IntakeState.PRELOADS, INTAKE_PRELOADS);
    }

    public void intakePreloads(double power) {
        setState(IntakeState.PRELOADS, power);
    }

    public String getState() {
        return state.name();
    }

    public boolean atState(IntakeState state) {
        return state == this.state;
    }

    private void setState(IntakeState nextState, double power) {
        boolean changed = state != nextState || Math.abs(getPower() - power) > 0.001;
        setPower(power);
        state = nextState;
        if (changed) {
            DecodeLogger.get().info("intake", "INTAKE_STATE_SET",
                    "state", nextState.name(),
                    "power", power);
        }
    }
}
