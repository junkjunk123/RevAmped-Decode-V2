package org.firstinspires.ftc.teamcode.utils.commands;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;

import org.firstinspires.ftc.teamcode.utils.logging.DecodeLogger;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

public class SimpleStateMachine<T> extends StateMachine<T> {
    private final AtomicInteger abortCounter = new AtomicInteger();
    private final String logTag;

    public SimpleStateMachine(T initialState) {
        this(initialState, "state");
    }

    public SimpleStateMachine(T initialState, String logTag) {
        super(initialState);
        this.logTag = logTag;
    }

    public ICommand runTransition(ICommand transition, Supplier<T> newState) {
        return new Sequential(
                new Instant(() -> {
                    if (currentGraphElement instanceof Edge edge) {
                        DecodeLogger.get().debug(logTag, "STATE_TRANSITION_ABORT",
                                "pending", String.valueOf(edge.nextState));
                        abortCounter.incrementAndGet();
                    }
                }),
                run(transition, newState)
        );
    }

    private ICommand run(ICommand transition, Supplier<T> newState) {
        AtomicInteger current = new AtomicInteger();
        return new Race(
                new Sequential(
                        new Instant(() -> {
                            currentGraphElement = new Edge(newState.get(), transition);
                            current.set(abortCounter.get());
                        }),
                        transition,
                        new Instant(() -> setCurrentState(newState.get()))
                ),
                new WaitUntil(() -> abortCounter.get() > current.get())
        );
    }

}
