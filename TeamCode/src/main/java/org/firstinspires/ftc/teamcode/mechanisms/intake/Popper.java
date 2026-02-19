package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.commands.SimpleStateMachine;
import org.firstinspires.ftc.teamcode.utils.commands.StateMachine;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;
import org.firstinspires.ftc.teamcode.utils.logging.DecodeLogger;

public class Popper extends HwServo {
    public static float NEUTRAL;
    public static float POP;
    public static float BLOCK;

    public enum PopperState {
        NEUTRAL,
        POP
    }
    private final StateMachine<PopperState> stateMachine = new SimpleStateMachine<>(PopperState.NEUTRAL, "popper");

    public Popper(HardwareMap hwMap) {
        super(hwMap, "popper");
    }

    public ICommand pop() {
        return new Lazy(() -> {
            if (!movingToState(PopperState.POP)) {
                return stateMachine.runTransition(
                        new Sequential(
                                new Instant(() -> setStateAndLog(POP, PopperState.POP, null)),
                                new Wait(250)
                        ),
                        PopperState.POP
                );
            }

            return Commands.NOOP;
        });
    }

    public ICommand block() {
        return new Sequential(
                new Instant(() -> setStateAndLog(BLOCK, PopperState.POP, "BLOCK")),
                new Wait(250),
                new Instant(() -> stateMachine.setCurrentState(PopperState.POP))
        );
    }

    public ICommand blockFromPop() {
        return new Sequential(
                new Instant(() -> setPosition(BLOCK)),
                new Wait(150),
                new Instant(() -> stateMachine.setCurrentState(PopperState.POP))
        );
    }

    public ICommand neutral() {
        return new Lazy(() -> {
            if (!movingToState(PopperState.NEUTRAL)) {
                return stateMachine.runTransition(
                        new Sequential(
                                new Instant(() -> setStateAndLog(NEUTRAL, PopperState.NEUTRAL, null)),
                                new Wait(500)
                        ),
                        PopperState.NEUTRAL
                );
            }

            return Commands.NOOP;
        });
    }

    public boolean atState(PopperState popperState) {
        return stateMachine.getCurrentState().equals(popperState);
    }

    public boolean movingToState(PopperState popperState) {
        return stateMachine.getPendingState().equals(popperState);
    }

    public void popCommandless() {
        setStateAndLog(POP, PopperState.POP, "COMMANDLESS");
        stateMachine.setCurrentState(PopperState.POP);
    }

    public String getState() {
        return stateMachine.getPendingState().name();
    }

    private void setStateAndLog(double position, PopperState nextState, String mode) {
        boolean moved = setPosition(position);
        boolean stateChanged = !stateMachine.getPendingState().equals(nextState);
        if (!moved && !stateChanged) return;

        if (mode == null) {
            DecodeLogger.get().info("popper", "POPPER_STATE_SET", "state", nextState.name());
        } else {
            DecodeLogger.get().info("popper", "POPPER_STATE_SET", "state", nextState.name(), "mode", mode);
        }
    }
}
