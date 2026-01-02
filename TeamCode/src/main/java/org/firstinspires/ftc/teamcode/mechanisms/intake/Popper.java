package org.firstinspires.ftc.teamcode.mechanisms.intake;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commands.SimpleStateMachine;
import org.firstinspires.ftc.teamcode.utils.commands.StateMachine;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class Popper extends HwServo {
    public static float NEUTRAL;
    public static float POP;

    public enum PopperState {
        NEUTRAL,
        POP
    }
    private final StateMachine<PopperState> stateMachine = new SimpleStateMachine<>(PopperState.NEUTRAL);

    public Popper(HardwareMap hwMap) {
        super(hwMap, "popper");
    }

    public ICommand pop() {
        return stateMachine.runTransition(
                new Sequential(
                        new Instant(() -> setPosition(POP)),
                        new Wait(250)
                ),
                PopperState.POP
        );
    }

    public ICommand neutral() {
        return stateMachine.runTransition(
                new Sequential(
                        new Instant(() -> setPosition(NEUTRAL)),
                        new Wait(250)
                ),
                PopperState.NEUTRAL
        );
    }

    public boolean atState(PopperState popperState) {
        return stateMachine.getCurrentState().equals(popperState);
    }

    public boolean movingToState(PopperState popperState) {
        return stateMachine.getPendingState().equals(popperState);
    }

    public String getState() {
        return stateMachine.getPendingState().name();
    }
}
