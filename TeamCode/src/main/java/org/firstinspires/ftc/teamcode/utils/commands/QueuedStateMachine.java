package org.firstinspires.ftc.teamcode.utils.commands;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;

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
    public ICommand runTransition(ICommand transition, Supplier<T> newState) {
        return new Sequential(
                new Instant(() -> enqueue(new Edge(newState.get(), transition))),
                new WaitUntil(() -> currentGraphElement instanceof Node),
                runAll()
        );
    }

    private ICommand runOne() {
        return new Sequential(
                new Instant(() -> {
                    Edge e = commandQueue.poll();
                    if (e != null) currentGraphElement = e;
                }),
                new Lazy(() -> ((Edge) currentGraphElement).command),
                new Instant(() -> setCurrentState(((Edge) currentGraphElement).nextState))
        );
    }

    private ICommand runAll() {
        return new Conditional(
                () -> commandQueue.isEmpty() || executing,
                new Command(),
                new Loop(runOne(), commandQueue::size)
                        .with(new Instant(() -> executing = true))
                        .then(new Instant(() -> executing = false))
        );
    }

    private void enqueue(Edge edge) {
        if (maxQueueSize < 0 || commandQueue.size() < maxQueueSize)
            commandQueue.offer(edge);
    }

    @Override
    public T getPendingState() {
        return !commandQueue.isEmpty() ? commandQueue.peekLast().nextState : super.getPendingState();
    }

    public AtomicReadOnce<T> pendingStateReader() {
        return new AtomicReadOnce<>(this::getPendingState);
    }
}
