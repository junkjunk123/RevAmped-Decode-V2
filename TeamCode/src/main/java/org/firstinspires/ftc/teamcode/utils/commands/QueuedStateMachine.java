package org.firstinspires.ftc.teamcode.utils.commands;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;

import org.firstinspires.ftc.teamcode.utils.AtomicReadOnce;

import java.util.ArrayDeque;
import java.util.function.Supplier;

public class QueuedStateMachine<T> extends StateMachine<T> {
    private final ArrayDeque<Edge> commandQueue = new ArrayDeque<>();
    private final int maxQueueSize;
    private boolean executing = false;

    public QueuedStateMachine(T initialState) {
        super(initialState);
        maxQueueSize = -1;
    }

    public QueuedStateMachine(T initialState, int maxQueueSize) {
        super(initialState);
        this.maxQueueSize = maxQueueSize;
    }

    @Override
    public CommandBuilder runTransition(CommandBuilder transition, Supplier<T> newState) {
        return sequential(
                instant(() -> enqueue(new Edge(newState.get(), transition))),
                waitUntil(() -> currentGraphElement instanceof Node),
                runAll()
        );
    }

    private CommandBuilder runOne() {
        return sequential(
                instant(() -> {
                    Edge e = commandQueue.poll();
                    if (e != null) currentGraphElement = e;
                }),
                lazy(() -> ((Edge) currentGraphElement).command),
                instant(() -> setCurrentState(((Edge) currentGraphElement).nextState))
        );
    }

    private CommandBuilder runAll() {
        return conditional(
                () -> commandQueue.isEmpty() || executing,
                Command.NOOP,
                RevAmpedCommands.loop(runOne(), commandQueue::size)
                        .with(instant(() -> executing = true))
                        .then(instant(() -> executing = false))
        );
    }

    private void enqueue(Edge edge) {
        if (maxQueueSize < 0 || commandQueue.size() < maxQueueSize)
            commandQueue.offer(edge);
    }

    @Override
    public T getPendingState() {
        T returnVal = !commandQueue.isEmpty() ? commandQueue.peekLast().nextState : super.getPendingState();
        return returnVal == null? super.getPendingState() : returnVal;
    }

    public AtomicReadOnce<T> pendingStateReader() {
        return new AtomicReadOnce<>(this::getPendingState);
    }
}
