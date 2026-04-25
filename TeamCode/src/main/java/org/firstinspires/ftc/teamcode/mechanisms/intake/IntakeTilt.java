package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class IntakeTilt extends HwServo {
    public static double INTAKE;
    public static double TRANSFER;
    public static double GATE_INTAKE;

    public enum TiltState {
        INTAKE,
        TRANSFER,
        GATE_INTAKE
    }
    private TiltState state = TiltState.INTAKE;

    public IntakeTilt(HardwareMap hardwareMap) {
        super(hardwareMap, "intakeTilt");
    }

    public void intake() {
        setPosition(INTAKE);
        state = TiltState.INTAKE;
    }

    public void transfer() {
        setPosition(TRANSFER);
        state = TiltState.TRANSFER;
    }

    public void gateIntake() {
        setPosition(GATE_INTAKE);
        state = TiltState.GATE_INTAKE;
    }

    public boolean atState(TiltState state) {
        return this.state.equals(state);
    }

    public String getState() {
        return state.name();
    }
}
