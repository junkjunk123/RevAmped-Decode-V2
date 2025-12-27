package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.pedropathing.math.Matrix;

import org.firstinspires.ftc.teamcode.RobotStateHandler;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Optional;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.stream.Collectors;

/**
 * TeleOpStateHandler is a class that manages the state transitions in a teleoperated mode of a robot.
 * It uses a finite state machine approach to handle transitions between different states.
 * The states are represented as an enum that implements the State interface.
 *
 */
public class TeleOpStateHandler {
    /**
     * The current graph element representing the state or transition in the state machine.
     */
    private GraphElement currentGraphElement;

    /**
     * The adjacency matrix representing the transitions between states.
     * Each entry in the matrix indicates whether a transition is valid (1) or not (0).
     */
    private Matrix adjMatrix;

    private final HashMap<RobotStateHandler.CycleState, Integer> states;

    private final Consumer<RobotStateHandler.Message> stateMutator;

    /**
     * The next transition operator that is pending execution.
     * This is used to handle transitions that are not immediately executed.
     */
    private ICommand next = null;

    /**
     * This flag indicates whether to force transitions regardless of the current state.
     */
    private boolean force = false;

    /**
     * GraphElement is an interface that represents an element in the state machine graph.
     */

    private int abortCounter;

    public interface GraphElement {

        /**
         * Gets the current state of this graph element.
         * @return the current state of this graph element
         */
        RobotStateHandler.CycleState getCurrentState();
    }

    /**
     * Transition is a record that represents a transition between two states in the state machine.
     * It implements the GraphElement interface and contains the current state, next state, and the transition command.
     */
    public record Transition(RobotStateHandler.CycleState current, RobotStateHandler.CycleState next, ICommand transitionCommand) implements GraphElement {
        /**
         * Gets the current state, before the transition.
         * @return the current state of this transition
         */
        @Override
        public RobotStateHandler.CycleState getCurrentState() {
            return current;
        }

        /**
         * Checks if the transition command is currently running.
         * @return true if the transition command is running, false otherwise
         */
        public boolean isRunning() {
            return !transitionCommand.done();
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof Transition that)) return false;
            return Objects.equals(next, that.next) && Objects.equals(current, that.current);
        }

        @Override
        public int hashCode() {
            return Objects.hash(current, next);
        }
    }

    /**
     * Gets the adjacency matrix representing the transitions between states.
     * @return the adjacency matrix
     */
    public Matrix getAdjMatrix() {
        return adjMatrix;
    }

    /**
     * Builds the adjacency matrix based on the transition vectors of the states in the enum class.
     * Each state is represented by a row in the matrix, and each column represents a possible transition to another state.
     */
    private void buildAdjMatrix() {
        List<RobotStateHandler.CycleState> states = new ArrayList<>(this.states.keySet());
        double[][] entries = new double[states.size()][states.size()];

        for (int i = 0; i < states.size(); i++) {
            entries[i] = states.get(i).getTransitionVector();
        }

        adjMatrix = new Matrix(entries);
    }

    /**
     * Constructor for TeleOpStateHandler.
     * Initializes the state handler with the initial state, scheduler, and builds the adjacency matrix.
     * @param initialState the initial state of the teleoperated mode
     */
    public TeleOpStateHandler(RobotStateHandler.CycleState initialState, HashMap<RobotStateHandler.CycleState, Integer> states, Consumer<RobotStateHandler.Message> mutateState) {
        this.currentGraphElement = initialState;
        this.states = states;
        stateMutator = mutateState;
        buildAdjMatrix();
    }

    /**
     * Sets a transition in the adjacency matrix.
     * @param row the row index representing the current state
     * @param col the column index representing the next state
     * @param transition a boolean indicating whether the transition is valid (true) or not (false)
     */
    public void setTransition(int row, int col, boolean transition) {
        adjMatrix.set(row, col, transition ? 1 : 0);
    }

    /**
     * Evaluates whether a transition from the current state to the next state is valid based on the adjacency matrix.
     * @param current the current state
     * @param next the next state
     * @return true if the transition is valid, false otherwise
     */
    public boolean evaluate(RobotStateHandler.CycleState current, RobotStateHandler.CycleState next) {
        return adjMatrix.get(states.get(current), states.get(next)) == 1;
    }

    /**
     * Sets the current state of the teleoperated mode.
     * @param currentState the new current state to be set
     */
    public void setCurrentState(RobotStateHandler.Message currentState) {
        this.currentGraphElement = currentState.cycleState();
        stateMutator.accept(currentState);
    }

    public void setCurrentState(RobotStateHandler.Message currentState, boolean init) {
        this.currentGraphElement = currentState.cycleState();
        if (init) stateMutator.accept(currentState);
    }

    /**
     * Runs a transition command to change the current state to the next state.
     * If the transition is valid, it executes the command and updates the current state.
     * If the transition is not valid, it checks if there is a pending transition and retries it after that transition is complete.
     * @param command the transition command to be executed
     * @param nextState the next state to transition to
     * @param force whether to force the transition regardless of the current state
     */
    public ICommand runTransition(ICommand command, RobotStateHandler.CycleState nextState, boolean force) {
        RobotStateHandler.CycleState currentState = currentState();
        LinkedHashMap<BooleanSupplier, ICommand> commandHashMap = new LinkedHashMap<>();

        commandHashMap.put(() -> (force || (currentGraphElement instanceof RobotStateHandler.CycleState &&
                        evaluate(currentState, nextState))), run(command, nextState, currentState));
        commandHashMap.put(() -> {
            if (next == null && currentGraphElement instanceof Transition transition) {
                RobotStateHandler.CycleState state = transition.next();
                return evaluate(state, nextState);
            }

            return false;
        }, new Sequential(
                new Instant(() -> next = command),
                new WaitUntil(() -> currentGraphElement instanceof RobotStateHandler.CycleState),
                new Instant(() -> next = null),
                retry(command, nextState, force)
        ));

        return new Optional(commandHashMap);
    }

    public ICommand runTransition(Runnable transition, RobotStateHandler.CycleState nextState, boolean force) {
        return runTransition(new Instant(transition), nextState, force);
    }

    /**
     * Retries a transition command if the current state is valid for the next state.
     * @param command the transition command to be executed
     * @param nextState the next state to transition to
     * @param force whether to force the transition regardless of the current state
     */
    private ICommand retry(ICommand command, RobotStateHandler.CycleState nextState, boolean force) {
        RobotStateHandler.CycleState currentState = currentState();

        return new Conditional(
                () -> force || (currentGraphElement instanceof RobotStateHandler.CycleState && evaluate(currentState, nextState)),
                run(command, nextState, currentState),
                new Command()
        );
    }

    /**
     * Runs a transition command and updates the current graph element to a new Transition.
     * It also schedules a delay to set the current state after the command is complete.
     * If a timeout is specified, it sets the maximum time to run the command.
     * @param command the transition command to be executed
     * @param nextState the next state to transition to
     * @param currentState the current state before the transition
     */
    private ICommand run(ICommand command, RobotStateHandler.CycleState nextState, RobotStateHandler.CycleState currentState) {
        AtomicInteger abortCounter = new AtomicInteger(0);
        return new Race (
                new Sequential(
                        new Instant(() -> currentGraphElement = new Transition(currentState, nextState, command)),
                        command,
                        new Instant(() -> setCurrentState(nextState))
                ),
                new Sequential(
                        new Instant(() -> abortCounter.set(this.abortCounter)),
                        new WaitUntil(() -> this.abortCounter - abortCounter.get() > 0)
                )
        );
    }

    /**
     * Runs a transition command to change the current state to the next state.
     * This method allows for a force flag to override the current state.
     * @param command the transition command to be executed
     * @param nextState the next state to transition to
     */
    public ICommand runTransition(ICommand command, RobotStateHandler.CycleState nextState) {
        return runTransition(command, nextState, force);
    }

    public ICommand runTransition(Runnable transition, RobotStateHandler.CycleState nextState) {
        return runTransition(new Instant(transition), nextState);
    }

    /**
     * Gets the current state of the teleoperated mode.
     * @return the current state of the teleoperated mode
     */
    public RobotStateHandler.CycleState currentState() {
        return currentGraphElement.getCurrentState();
    }

    /**
     * Gets the current graph element representing the state or transition in the state machine.
     * @return the current graph element
     */
    public GraphElement currentElement() {
        return currentGraphElement;
    }

    /**
     * Sets a setting to be executed after a delay, with dependencies on other graph elements.
     * @param setting the setting to be executed
     * @param dependencies the list of graph elements that this setting depends on
     */
    public ICommand setting(ICommand setting, List<GraphElement> dependencies) {
        LinkedHashMap<BooleanSupplier, ICommand> commandHashMap = new LinkedHashMap<>();
        commandHashMap.put(() -> force, setting);
        commandHashMap.put(() -> !force, new Sequential(
                new WaitUntil(() -> !dependencies.contains(currentGraphElement)),
                setting
        ));

        return new Optional(commandHashMap);
    }

    public ICommand setting(Runnable setting, List<GraphElement> dependencies) {
        return setting(new Instant(setting), dependencies);
    }

    /**
     * Executes this setting after the current transition is completed, or immediately if the current graph element isn't a transition.
     */
    public ICommand setting(ICommand setting) {
        LinkedHashMap<BooleanSupplier, ICommand> commandHashMap = new LinkedHashMap<>();
        commandHashMap.put(() -> force, setting);
        commandHashMap.put(() -> currentGraphElement instanceof RobotStateHandler.CycleState, setting);
        commandHashMap.put(() -> !force && !(currentGraphElement instanceof RobotStateHandler.CycleState), new Sequential(
                new WaitUntil(() -> currentElement() instanceof RobotStateHandler.CycleState),
                setting
        ));

        return new Optional(commandHashMap);
    }

    public ICommand setting(Runnable runnable) {
        return setting(new Instant(runnable));
    }

    /**
     * Executes a task if the current state matches the component vector.
     * The component vector is a binary vector that indicates which components are active in the current state.
     * @param task the task to be executed
     * @param componentVector the binary vector representing the active components in the current state
     */
    public ICommand task(ICommand task, int[] componentVector) {
        LinkedHashMap<BooleanSupplier, ICommand> commandHashMap = new LinkedHashMap<>();
        commandHashMap.put(() -> force, task);
        commandHashMap.put(() -> currentGraphElement instanceof RobotStateHandler.CycleState &&
                componentVector[states.get(currentState())] == 1, task);
        commandHashMap.put(() -> currentGraphElement instanceof Transition &&
                componentVector[states.get(((Transition) currentGraphElement).next())] == 1,
                new Sequential(
                        new WaitUntil(() -> !(currentGraphElement instanceof Transition)),
                        task
                )
        );

        return new Optional(commandHashMap);
    }

    public ICommand task(Runnable task, int[] componentVector) {
        return task(new Instant(task), componentVector);
    }

    /**
     * Executes a task if the current state matches the specified state.
     * @param task the task to be executed
     * @param state the state to check against the current state
     */
    public ICommand task(ICommand task, RobotStateHandler.CycleState state) {
        LinkedHashMap<BooleanSupplier, ICommand> commandHashMap = new LinkedHashMap<>();
        commandHashMap.put(() -> force, task);
        commandHashMap.put(() -> currentGraphElement instanceof RobotStateHandler.CycleState && currentState() == state, task);
        commandHashMap.put(() -> currentGraphElement instanceof Transition transition && transition.next == state,
                new Sequential(
                        new WaitUntil(((Transition) currentGraphElement).transitionCommand::done),
                        task
                )
        );
        return new Optional(commandHashMap);
    }

    public ICommand task(Runnable task, RobotStateHandler.CycleState state) {
        return task(new Instant(task), state);
    }

    /**
     * Sets whether to force transitions regardless of the current state.
     * @param force true to force transitions, false otherwise
     */
    public void setForce(boolean force) {
        this.force = force;
    }

    /**
     * Checks if transitions are forced regardless of the current state.
     * @return true if transitions are forced, false otherwise
     */
    public boolean isForce() {
        return force;
    }

    public RobotStateHandler.CycleState nextState() {
        if (currentGraphElement instanceof RobotStateHandler.CycleState state) {
            return state;
        }

        else if (currentGraphElement instanceof Transition transition) {
            return transition.next;
        }

        return null;
    }

    public static Transition referenceTransition(RobotStateHandler.CycleState current, RobotStateHandler.CycleState next) {
        return new Transition(current, next, new Instant(() -> {}));
    }

    public boolean atState(RobotStateHandler.CycleState state) {
        return currentState().equals(state);
    }

    public boolean nextStateAt(RobotStateHandler.CycleState state) {
        return nextState().equals(state);
    }

    /**
     * Aborts the current transition if it is a Transition instance.
     * This method sets the transition command to aborted, allowing for a clean exit from the transition.
     */
    public void abortTransition() {
        if (currentGraphElement instanceof Transition)
            abortCounter++;
    }

    /**
     * Overrides the current transition with a new action and sets the next state.
     * This method allows for a clean override of the current transition, ensuring that the new action is executed
     * and the current state is updated to the next state.
     * @param overrideAction the action to be executed as an override
     * @param nextState the next state to transition to after the override
     */
    public void override(Runnable overrideAction, RobotStateHandler.CycleState nextState) {
        abortTransition();
        overrideAction.run();
        setCurrentState(nextState);
    }

    public int getAbortCounter() {
        return abortCounter;
    }
}