package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.pedropathing.math.Matrix;

import org.firstinspires.ftc.teamcode.utils.commands.OptionCommand;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * TeleOpStateHandler is a class that manages the state transitions in a teleoperated mode of a robot.
 * It uses a finite state machine approach to handle transitions between different states.
 * The states are represented as an enum that implements the State interface.
 *
 * @param <T> the type of the enum representing the states
 */
public class TeleOpStateHandler<T extends TeleOpStateHandler.State> {
    /**
     * The current graph element representing the state or transition in the state machine.
     */
    private GraphElement currentGraphElement;

    /**
     * The adjacency matrix representing the transitions between states.
     * Each entry in the matrix indicates whether a transition is valid (1) or not (0).
     */
    private Matrix adjMatrix;

    /**
     * The class of the states.
     */
    private final Class<T> stateClass;

    private final HashMap<T, Integer> states;

    private final Consumer<T> stateMutator;

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
        State getCurrentState();
    }

    /**
     * State is an interface that represents a state in the teleoperated mode of the robot.
     */
    public interface State extends GraphElement {
        /**
         * Gets the transition vector for this state.
         * The transition vector is a binary vector that represents the possible transitions from this state.
         * @return a binary vector representing the transition vector
         */
        double[] getTransitionVector();

        /**
         * Gets the next state that this state can transition to.
         * @return the next state that this state can transition to
         */
        @Override
        default State getCurrentState() {
            return this;
        }
    }

    /**
     * Transition is a record that represents a transition between two states in the state machine.
     * It implements the GraphElement interface and contains the current state, next state, and the transition command.
     */
    public record Transition(State current, State next, ICommand transitionCommand) implements GraphElement {
        /**
         * Gets the current state, before the transition.
         * @return the current state of this transition
         */
        @Override
        public State getCurrentState() {
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
        T[] states = stateClass.getEnumConstants();
        assert states != null;
        double[][] entries = new double[states.length][states.length];

        for (int i = 0; i < states.length; i++) {
            entries[i] = states[i].getTransitionVector();
        }

        adjMatrix = new Matrix(entries);
    }

    /**
     * Constructor for TeleOpStateHandler.
     * Initializes the state handler with the initial state, scheduler, and builds the adjacency matrix.
     * @param initialState the initial state of the teleoperated mode
     */
    public TeleOpStateHandler(T initialState, Class<T> stateClass, HashMap<T, Integer> states, Consumer<T> mutateState) {
        this.currentGraphElement = initialState;
        this.stateClass = stateClass;
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
    public boolean evaluate(T current, T next) {
        return adjMatrix.get(states.get(current), states.get(next)) == 1;
    }

    /**
     * Sets the current state of the teleoperated mode.
     * @param currentState the new current state to be set
     */
    public void setCurrentState(T currentState) {
        this.currentGraphElement = currentState;
        stateMutator.accept(currentState);
    }

    /**
     * Runs a transition command to change the current state to the next state.
     * If the transition is valid, it executes the command and updates the current state.
     * If the transition is not valid, it checks if there is a pending transition and retries it after that transition is complete.
     * @param command the transition command to be executed
     * @param nextState the next state to transition to
     * @param force whether to force the transition regardless of the current state
     */
    public ICommand runTransition(ICommand command, T nextState, boolean force) {
        T currentState = currentState();
        LinkedHashMap<BooleanSupplier, ICommand> commandHashMap = new LinkedHashMap<>();

        commandHashMap.put(() -> (force || currentGraphElement instanceof State) &&
                        evaluate(currentState, nextState), run(command, nextState, currentState));
        commandHashMap.put(() -> {
            if (next == null && currentGraphElement instanceof Transition transition) {
                State state = transition.next();
                if (!stateClass.isInstance(state)) {
                    throw new IllegalStateException("Current state is not of expected enum type.");
                }
                T pendingNext = stateClass.cast(state);
                assert pendingNext != null;
                return evaluate(pendingNext, nextState);
            }

            return false;
        }, new Sequential(
                new Instant(() -> next = command),
                new WaitUntil(() -> currentGraphElement instanceof State),
                new Instant(() -> next = null),
                runTransition(command, nextState, force)
        ));

        return new OptionCommand(commandHashMap);
    }

    public ICommand runTransition(Runnable transition, T nextState, boolean force) {
        return runTransition(new Instant(transition), nextState, force);
    }

    /**
     * Runs a transition command and updates the current graph element to a new Transition.
     * It also schedules a delay to set the current state after the command is complete.
     * If a timeout is specified, it sets the maximum time to run the command.
     * @param command the transition command to be executed
     * @param nextState the next state to transition to
     * @param currentState the current state before the transition
     */
    private ICommand run(ICommand command, T nextState, T currentState) {
        int abortCounter = this.abortCounter;
        return new Race (
                new Sequential(
                        new Instant(() -> currentGraphElement = new Transition(currentState, nextState, command)),
                        command,
                        new Instant(() -> setCurrentState(nextState))
                ),
                new Sequential(
                        new WaitUntil(() -> this.abortCounter - abortCounter > 0)
                )
        );
    }

    /**
     * Runs a transition command to change the current state to the next state.
     * This method allows for a force flag to override the current state.
     * @param command the transition command to be executed
     * @param nextState the next state to transition to
     */
    public ICommand runTransition(ICommand command, T nextState) {
        return runTransition(command, nextState, force);
    }

    public ICommand runTransition(Runnable transition, T nextState) {
        return runTransition(new Instant(transition), nextState);
    }

    /**
     * Gets the current state of the teleoperated mode.
     * @return the current state of the teleoperated mode
     */
    public T currentState() {
        State state = currentGraphElement.getCurrentState();
        if (!stateClass.isInstance(state)) {
            throw new IllegalStateException("Current state is not of expected enum type.");
        }
        return stateClass.cast(state);
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

        return new OptionCommand(commandHashMap);
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
        commandHashMap.put(() -> currentGraphElement instanceof State, setting);
        commandHashMap.put(() -> !force && !(currentGraphElement instanceof State), new Sequential(
                new WaitUntil(() -> currentElement() instanceof State),
                setting
        ));

        return new OptionCommand(commandHashMap);
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
        commandHashMap.put(() -> currentGraphElement instanceof State &&
                componentVector[states.get(currentState())] == 1, task);
        commandHashMap.put(() -> currentGraphElement instanceof Transition transition &&
                componentVector[Objects.requireNonNull(states.get(stateClass.cast(transition.next())))] == 1,
                new Sequential(
                        new WaitUntil(((Transition) currentGraphElement).transitionCommand::done),
                        task
                )
        );

        return new OptionCommand(commandHashMap);
    }

    public ICommand task(Runnable task, int[] componentVector) {
        return task(new Instant(task), componentVector);
    }

    /**
     * Executes a task if the current state matches the specified state.
     * @param task the task to be executed
     * @param state the state to check against the current state
     */
    public ICommand task(ICommand task, T state) {
        LinkedHashMap<BooleanSupplier, ICommand> commandHashMap = new LinkedHashMap<>();
        commandHashMap.put(() -> force, task);
        commandHashMap.put(() -> currentGraphElement instanceof State && currentState() == state, task);
        commandHashMap.put(() -> currentGraphElement instanceof Transition transition && stateClass.cast(transition.next) == state,
                new Sequential(
                        new WaitUntil(((Transition) currentGraphElement).transitionCommand::done),
                        task
                )
        );
        return new OptionCommand(commandHashMap);
    }

    public ICommand task(Runnable task, T state) {
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

    public T nextState() {
        if (currentGraphElement instanceof State state) {
            try {
                return stateClass.cast(state);
            } catch (Exception ignored) {
                throw new RuntimeException();
            }
        }

        else if (currentGraphElement instanceof Transition transition) {
            try {
                return stateClass.cast(transition.next);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        return null;
    }

    public static Transition referenceTransition(State current, State next) {
        return new Transition(current, next, new Instant(() -> {}));
    }

    public boolean atState(T state) {
        return currentState().equals(state);
    }

    public boolean nextStateAt(T state) {
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
    public void override(Runnable overrideAction, T nextState) {
        abortTransition();
        overrideAction.run();
        setCurrentState(nextState);
    }

    public int getAbortCounter() {
        return abortCounter;
    }
}