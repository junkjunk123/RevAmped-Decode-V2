package org.firstinspires.ftc.teamcode.mechanisms;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.math.Matrix;

import java.util.ArrayList;
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
            CommandBuilder command
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
            CommandBuilder command,
            RobotStateHandler.CycleState next,
            boolean force
    ) {}

    private GraphElement current;
    private TransitionRequest queued;
    private Matrix adj;
    private final ArrayList<RobotStateHandler.CycleState> index;
    private final Consumer<RobotStateHandler.Message> mutator;
    private final AtomicInteger abortCounter = new AtomicInteger();
    private boolean force;

    public TeleOpStateHandler(
            RobotStateHandler.CycleState initial,
            ArrayList<RobotStateHandler.CycleState> index,
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

    public CommandBuilder runTransition(
            CommandBuilder command,
            RobotStateHandler.CycleState next
    ) {
        return runTransition(command, next, force);
    }

    public CommandBuilder runTransition(Runnable command, RobotStateHandler.CycleState next) {
        return runTransition(instant(command), next);
    }

    public CommandBuilder runTransition(
            CommandBuilder command,
            RobotStateHandler.CycleState next,
            boolean force
    ) {
        return conditional(
                () -> canStart(next, force),
                start(command, next),
                queue(command, next, force)
        );
    }

    public CommandBuilder runTransition(Runnable command, RobotStateHandler.CycleState next, boolean force) {
        return runTransition(instant(command), next, force);
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

    private CommandBuilder start(
            CommandBuilder command,
            RobotStateHandler.CycleState next
    ) {
        AtomicInteger snapshot = new AtomicInteger();

        return race(
                sequential(
                        instant(() ->
                                current = new Transition(currentState(), next, command)
                        ),
                        command,
                        instant(() -> setState(next)),
                        consumeQueued()
                ),
                sequential(
                        instant(() ->
                                snapshot.set(abortCounter.get())
                        ),
                        waitUntil(() ->
                                abortCounter.get() > snapshot.get()
                        )
                )
        );
    }

    private CommandBuilder queue(
            CommandBuilder command,
            RobotStateHandler.CycleState next,
            boolean force
    ) {
        return instant(() -> {
            if (current instanceof Transition t && queued == null)
                if (force || valid(t.to(), next)) queued = new TransitionRequest(command, next, force);
            }
        );
    }

    private CommandBuilder consumeQueued() {
        return lazy(() -> {
            if (queued != null) {
                TransitionRequest r = queued;
                queued = null;
                return start(r.command(), r.next);
            }

            return Command.NOOP;
        });
    }

    private void setState(RobotStateHandler.CycleState state) {
        current = state;
        mutator.accept(state);
    }

    public boolean valid(
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

    public CommandBuilder setting(CommandBuilder setting) {
        return conditional(
                () -> force || current instanceof RobotStateHandler.CycleState,
                setting,
                sequential(
                        waitUntil(() ->
                                current instanceof RobotStateHandler.CycleState
                        ),
                        setting
                )
        );
    }

    public CommandBuilder setting(
            CommandBuilder setting,
            List<GraphElement> dependencies
    ) {
        return conditional(
                () -> force,
                setting,
                sequential(
                        waitUntil(() ->
                                !dependencies.contains(current)
                        ),
                        setting
                )
        );
    }

    public CommandBuilder task(
            CommandBuilder task,
            RobotStateHandler.CycleState state
    ) {
        return conditional(
                () -> force || currentState() == state,
                task,
                sequential(
                        waitUntil(() ->
                                currentState() == state
                        ),
                        task
                )
        );
    }

    public CommandBuilder task(
            CommandBuilder task,
            int[] componentVector
    ) {
        return conditional(
                () -> force ||
                        (current instanceof RobotStateHandler.CycleState &&
                                componentVector[index.indexOf(currentState())] == 1),
                task,
                sequential(
                        waitUntil(() ->
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

    public CommandBuilder setting(Runnable setting) {
        return setting(instant(setting));
    }

    public CommandBuilder task(Runnable task, int[] componentVector) {
        return task(instant(task), componentVector);
    }

    public CommandBuilder task(Runnable task, RobotStateHandler.CycleState state) {
        return task(instant(task), state);
    }

    public CommandBuilder override(CommandBuilder overrideAction, RobotStateHandler.Message nextState) {
        return sequential(
                instant(this::abortTransition),
                overrideAction,
                instant(() -> setState(nextState.cycleState()))
        );
    }

    public Matrix getAdj() {
        return adj;
    }
}
