package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.commands.SimpleStateMachine;
import org.firstinspires.ftc.teamcode.utils.commands.StateMachine;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class Popper extends HwServo {
    public static float NEUTRAL;
    public static float POP;
    public static float BLOCK;

    public enum PopperState {
        NEUTRAL,
        POP,
        BLOCK
    }
    private final StateMachine<PopperState> stateMachine = new SimpleStateMachine<>(PopperState.NEUTRAL);

    public Popper(HardwareMap hwMap) {
        super(hwMap, "popper");
    }

    public ICommand pop() {
        return new Lazy(() -> {
            if (!movingToState(PopperState.POP)) {
                return stateMachine.runTransition(
                        new Sequential(
                                new Instant(() -> setPosition(POP)),
                                new Wait(
                                        stateMachine.getPendingState().equals(PopperState.NEUTRAL) ?
                                        250 : 150
                                )
                        ),
                        PopperState.POP
                );
            }

            return new Instant(() -> setPosition(POP));
        });
    }

    public ICommand block() {
        return new Lazy(() -> {
            if (!movingToState(PopperState.BLOCK)) {
                return stateMachine.runTransition(
                        new Sequential(
                                new Instant(() -> setPosition(BLOCK)),
                                new Wait(
                                        stateMachine.getPendingState().equals(PopperState.NEUTRAL) ?
                                        250 : 150
                                )
                        ),
                        PopperState.BLOCK
                );
            }

            return new Instant(() -> setPosition(BLOCK));
        });
    }

    public ICommand neutral() {
        return new Lazy(() -> {
            if (!movingToState(PopperState.NEUTRAL)) {
                return moveToNeutral();
            }

            return new Instant(() -> setPosition(NEUTRAL));
        });
    }

    public ICommand moveToNeutral() {
        return stateMachine.runTransition(
                new Sequential(
                        new Instant(() -> setPosition(NEUTRAL)),
                        new Wait(
                                stateMachine.getPendingState().equals(PopperState.POP) ?
                                250 : 150
                        )
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

    public void popCommandless() {
        setPosition(POP);
        stateMachine.setCurrentState(PopperState.POP);
    }
    public void neutralCommandless() {
        setPosition(NEUTRAL);
        stateMachine.setCurrentState(PopperState.NEUTRAL);
    }


    public String getState() {
        return stateMachine.getPendingState().name();
    }
}