package org.firstinspires.ftc.teamcode.mechanisms.intake;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class Popper extends HwServo {
    public static float NEUTRAL;
    public static float POP;

    public enum PopperState {
        NEUTRAL,
        POP
    }
    private PopperState state = PopperState.NEUTRAL;

    public Popper(HardwareMap hwMap) {
        super(hwMap, "popper");
    }

    public void pop() {
        setPosition(POP);
        this.state = PopperState.POP;
    }

    public void neutral() {
        setPosition(NEUTRAL);
        this.state = PopperState.NEUTRAL;
    }

    public boolean atState(PopperState popperState) {
        return state == popperState;
    }

    public String getState() {
        return state.name();
    }
}
