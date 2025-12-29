package org.firstinspires.ftc.teamcode.utils.commands;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

public class SimpleStateMachine<T> extends StateMachine<T> {
    private final AtomicInteger abortCounter = new AtomicInteger();

    public SimpleStateMachine(T initialState) {
        super(initialState);
    }

    public ICommand runTransition(ICommand transition, Supplier<T> newState) {
        return new Conditional(
                () -> currentGraphElement instanceof Node,
                run(transition, newState),
                new Sequential(
                        new Instant(abortCounter::getAndIncrement),
                        run(transition, newState)
                )
        );
    }

    private ICommand run(ICommand transition, Supplier<T> newState) {
        AtomicInteger current = new AtomicInteger();
        return new Race(
                new Sequential(
                        new Instant(() -> {
                            currentGraphElement = new Edge(newState, transition);
                            current.set(abortCounter.get());
                        }),
                        transition,
                        new Instant(() -> setCurrentState(newState.get()))
                ),
                new WaitUntil(() -> abortCounter.get() > current.get())
        );
    }

}
