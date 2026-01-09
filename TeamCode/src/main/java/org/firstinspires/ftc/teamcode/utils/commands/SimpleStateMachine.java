package org.firstinspires.ftc.teamcode.utils.commands;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

public class SimpleStateMachine<T> extends StateMachine<T> {
    private final AtomicInteger abortCounter = new AtomicInteger();

    public SimpleStateMachine(T initialState) {
        super(initialState);
    }

    public Command runTransition(Command transition, Supplier<T> newState) {
        return conditional(
                () -> currentGraphElement instanceof Node,
                run(transition, newState),
                sequential(
                        instant(abortCounter::getAndIncrement),
                        run(transition, newState)
                )
        );
    }

    private Command run( Command transition, Supplier<T> newState) {
        AtomicInteger current = new AtomicInteger();
        return race(
                sequential(
                        instant(() -> {
                            currentGraphElement = new Edge(newState.get(), transition);
                            current.set(abortCounter.get());
                        }),
                        transition,
                        instant(() -> setCurrentState(newState.get()))
                ),
                waitUntil(() -> abortCounter.get() > current.get())
        );
    }

}
