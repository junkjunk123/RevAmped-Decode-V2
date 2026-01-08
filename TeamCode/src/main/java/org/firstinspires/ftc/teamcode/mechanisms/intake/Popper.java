package org.firstinspires.ftc.teamcode.mechanisms.intake;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commands.SimpleStateMachine;
import org.firstinspires.ftc.teamcode.utils.commands.StateMachine;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

import java.util.Objects;

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

    public CommandBuilder pop() {
        return lazy(() -> {
            if (!Objects.equals(getState(), PopperState.POP.name())) {
                return stateMachine.runTransition(
                        sequential(
                                instant(() -> setPosition(POP)),
                                Commands.wait(250.0)
                        ),
                        PopperState.POP
                );
            }

            return Command.NOOP;
        });
    }

    public CommandBuilder neutral() {
        return lazy(() -> {
            if (!Objects.equals(getState(), PopperState.NEUTRAL.name())) {
                return stateMachine.runTransition(
                        sequential(
                                instant(() -> setPosition(NEUTRAL)),
                                Commands.wait(250.0)
                        ),
                        PopperState.NEUTRAL
                );
            }

            return Command.NOOP;
        });
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
