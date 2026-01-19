package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.pedropathing.math.Matrix;

import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

/**
 * TeleOpStateHandler manages finite-state transitions during TeleOp.
 * Exactly one transition may run at a time, with at most one queued transition.
 */
public final class TeleOpStateHandler {

    public sealed interface GraphElement
            permits RobotStateHandler.CycleState, Transition {

        RobotStateHandler.CycleState getCurrentState();
    }

    public record Transition(
            RobotStateHandler.CycleState from,
            RobotStateHandler.CycleState to,
            ICommand command
    ) implements GraphElement {

        @Override
        public RobotStateHandler.CycleState getCurrentState() {
            return from;
        }

        public boolean running() {
            return !command.done();
        }

        @Override
        public boolean equals(Object o) {
            if (!(o instanceof Transition t)) return false;
            return from == t.from && to == t.to;
        }

        @Override
        public int hashCode() {
            return Objects.hash(from, to);
        }
    }

    private record TransitionRequest(
            ICommand command,
            RobotStateHandler.CycleState next,
            boolean force
    ) {}

    private GraphElement current;
    private TransitionRequest queued;
    private Matrix adj;
    private final List<RobotStateHandler.CycleState> index;
    private final Consumer<RobotStateHandler.Message> mutator;
    private final AtomicInteger abortCounter = new AtomicInteger();
    private boolean force;

    public TeleOpStateHandler(
            RobotStateHandler.CycleState initial,
            List<RobotStateHandler.CycleState> index,
            Consumer<RobotStateHandler.Message> mutator
    ) {
        this.current = initial;
        this.index = index;
        this.mutator = mutator;
        buildAdjacency();
    }

    public RobotStateHandler.CycleState currentState() {
        return current.getCurrentState();
    }

    public boolean atState(RobotStateHandler.CycleState state) {
        return currentState().equals(state);
    }

    public GraphElement currentElement() {
        return current;
    }

    public void setForce(boolean force) {
        this.force = force;
    }

    public void abortTransition() {
        if (current instanceof Transition)
            abortCounter.incrementAndGet();
    }

    public ICommand runTransition(
            ICommand command,
            RobotStateHandler.CycleState next
    ) {
        return runTransition(command, next, force);
    }

    public ICommand runTransition(Runnable command, RobotStateHandler.CycleState next) {
        return runTransition(new Instant(command), next);
    }

    public ICommand runTransition(
            ICommand command,
            RobotStateHandler.CycleState next,
            boolean force
    ) {
        return new Conditional(
                () -> canStart(next, force),
                start(command, next),
                queue(command, next, force)
        );
    }

    public ICommand runTransition(Runnable command, RobotStateHandler.CycleState next, boolean force) {
        return runTransition(new Instant(command), next, force);
    }

    public boolean evaluate(RobotStateHandler.CycleState next) {
        return canStart(next, force);
    }

    private boolean canStart(
            RobotStateHandler.CycleState next,
            boolean force
    ) {
        if (current instanceof RobotStateHandler.CycleState s) {
            return force || valid(s, next);
        }
        return false;
    }

    private ICommand start(
            ICommand command,
            RobotStateHandler.CycleState next
    ) {
        AtomicInteger snapshot = new AtomicInteger();

        return new Race(
                new Sequential(
                        new Instant(() ->
                                current = new Transition(currentState(), next, command)
                        ),
                        command,
                        new Instant(() -> setState(next)),
                        consumeQueued()
                ),
                new Sequential(
                        new Instant(() ->
                                snapshot.set(abortCounter.get())
                        ),
                        new WaitUntil(() ->
                                abortCounter.get() > snapshot.get()
                        )
                )
        );
    }

    private ICommand queue(
            ICommand command,
            RobotStateHandler.CycleState next,
            boolean force
    ) {
        return new Instant(() -> {
            if (current instanceof Transition t && queued == null)
                if (force || valid(t.to(), next)) queued = new TransitionRequest(command, next, force);
            }
        );
    }

    private ICommand consumeQueued() {
        return new Lazy(() -> {
            if (queued != null) {
                TransitionRequest r = queued;
                queued = null;
                return start(r.command(), r.next);
            }

            return Commands.NOOP;
        });
    }

    private void setState(RobotStateHandler.CycleState state) {
        current = state;
        mutator.accept(state);
    }

    private boolean valid(
            RobotStateHandler.CycleState from,
            RobotStateHandler.CycleState to
    ) {
        return adj.get(index.indexOf(from), index.indexOf(to)) == 1;
    }

    private void buildAdjacency() {
        double[][] m = new double[index.size()][index.size()];
        for (int i = 0; i < index.size(); i++)
            m[i] = index.get(i).getTransitionVector();
        adj = new Matrix(m);
    }

    public ICommand setting(ICommand setting) {
        return new Conditional(
                () -> force || current instanceof RobotStateHandler.CycleState,
                setting,
                new Sequential(
                        new WaitUntil(() ->
                                current instanceof RobotStateHandler.CycleState
                        ),
                        setting
                )
        );
    }

    public ICommand setting(
            ICommand setting,
            List<GraphElement> dependencies
    ) {
        return new Conditional(
                () -> force,
                setting,
                new Sequential(
                        new WaitUntil(() ->
                                !dependencies.contains(current)
                        ),
                        setting
                )
        );
    }

    public ICommand task(
            ICommand task,
            RobotStateHandler.CycleState state
    ) {
        return new Conditional(
                () -> force || currentState() == state,
                task,
                new Sequential(
                        new WaitUntil(() ->
                                currentState() == state
                        ),
                        task
                )
        );
    }

    public ICommand task(
            ICommand task,
            int[] componentVector
    ) {
        return new Conditional(
                () -> force ||
                        (current instanceof RobotStateHandler.CycleState &&
                                componentVector[index.indexOf(currentState())] == 1),
                task,
                new Sequential(
                        new WaitUntil(() ->
                                current instanceof RobotStateHandler.CycleState &&
                                        componentVector[index.indexOf(currentState())] == 1
                        ),
                        task
                )
        );
    }

    public boolean isForce() {
        return force;
    }

    public ICommand setting(Runnable setting) {
        return setting(new Instant(setting));
    }

    public ICommand task(Runnable task, int[] componentVector) {
        return task(new Instant(task), componentVector);
    }

    public ICommand task(Runnable task, RobotStateHandler.CycleState state) {
        return task(new Instant(task), state);
    }

    public ICommand override(ICommand overrideAction, RobotStateHandler.Message nextState) {
        return new Sequential(
                new Instant(this::abortTransition),
                overrideAction,
                new Instant(() -> setState(nextState.cycleState()))
        );
    }

    public Matrix getAdj() {
        return adj;
    }
}
