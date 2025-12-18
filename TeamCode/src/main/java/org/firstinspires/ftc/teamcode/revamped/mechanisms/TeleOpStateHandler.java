package org.firstinspires.ftc.teamcode.revamped.mechanisms;

import android.util.Log;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.pedropathing.math.Matrix;

import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;

/**
 * TeleOpStateHandler is a class that manages the state transitions in a teleoperated mode of a robot.
 * It uses a finite state machine approach to handle transitions between different states.
 * The states are represented as an enum that implements the State interface.
 *
 * @param <T> the type of the enum representing the states
 */
public class TeleOpStateHandler<T extends Enum<T> & TeleOpStateHandler.State> {
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
     * The class of the enum representing the states.
     */
    private final Class<T> enumClass;

    /**
     * The next transition operator that is pending execution.
     * This is used to handle transitions that are not immediately executed.
     */
    private TransitionOperator next = null;

    /**
     * This flag indicates whether to force transitions regardless of the current state.
     */
    private boolean force = false;

    /**
     * GraphElement is an interface that represents an element in the state machine graph.
     */
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
    public record Transition(State current, State next, TransitionOperator transitionCommand) implements GraphElement {
        /**
         * Gets the current state, before the transition.
         * @return the current state of this transition
         */
        @Override
        public State getCurrentState() {
            return current;
        }

        /**
         * Checks if the transition command is complete.
         * @return true if the transition command is complete, false otherwise
         */
        public boolean isComplete() {
            return transitionCommand.isComplete();
        }

        /**
         * Checks if the transition command has been aborted.
         * @return true if the transition command has been aborted, false otherwise
         */
        public boolean isAborted() {
            return transitionCommand.aborted();
        }

        /**
         * Checks if the transition command is currently running.
         * @return true if the transition command is running, false otherwise
         */
        public boolean isRunning() {
            return !isAborted() && !isComplete();
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
     * TransitionCommand is a functional interface that defines a command to be executed during a state transition.
     * It provides a method to run the command and an optional timeout for the command execution.
     */
    @FunctionalInterface
    public interface TransitionCommand {
        /**
         * Runs the transition command.
         * @param setComplete a consumer that is called to indicate whether the command is complete
         */
        void run(Consumer<Boolean> setComplete);

        /**
         * Gets the timeout for the transition command.
         * @return the timeout in seconds, or -1 if no timeout is set
         */
        default double timeout() {return -1;}

        /**
         * Creates a TransitionCommand with a specified timeout.
         * @param action the action to be executed as part of the transition command
         * @param timeout the timeout in seconds for the transition command
         * @return a TransitionCommand that executes the action and sets the completion status after the timeout
         */
        static TransitionCommand withTimeout(Consumer<Consumer<Boolean>> action, double timeout) {
            return new TransitionCommand() {
                @Override
                public void run(Consumer<Boolean> setComplete) {
                    action.accept(setComplete);
                }

                @Override
                public double timeout() {
                    return timeout;
                }
            };
        }

        /**
         * Creates a TransitionCommand that runs a specified action and sets the completion status to true.
         * @param action the action to be executed as part of the transition command
         * @return a TransitionCommand that executes the action and sets the completion status
         */
        static TransitionCommand fromRunnable(Runnable action) {
            return setComplete -> {
                action.run();
                setComplete.accept(true);
            };
        }
    }

    /**
     * TransitionOperator is a class that manages the execution of a transition command.
     * It keeps track of whether the command is complete, aborted, or still running.
     */
    public static class TransitionOperator {
        /**
         * The transition command to be executed.
         */
        private final TransitionCommand transition;
        public Boolean complete = false;
        private final Consumer<Boolean> setComplete = done -> {
            if (done == null || complete) return;
            complete = done;
        };

        /**
         * Constructor for TransitionOperator.
         * @param transition the transition command to be executed
         */
        public TransitionOperator(TransitionCommand transition) {
            this.transition = transition;
        }

        /**
         * Runs the transition command and sets the completion status.
         * If a timeout is specified, it schedules a delay to set the completion status after the timeout.
         */
        public void run() {
            if (transition.timeout() > 0) {
                Scheduler.getInstance().schedule(
                        new Sequential(
                                new Wait(transition.timeout()),
                                new Instant(() -> setComplete.accept(null))
                        )
                );
            }

            transition.run(setComplete);
        }

        /**
         * Checks if the transition command is complete.
         * @return true if the transition command is complete, false otherwise
         */
        public boolean isComplete() {
            return complete != null && complete;
        }

        /**
         * Checks if the transition command has been aborted.
         * If the command is not complete and the complete status is null, it is considered aborted.
         * @return true if the transition command has been aborted, false otherwise
         */
        public boolean aborted() {
            return !Boolean.FALSE.equals(complete) && complete == null;
        }

        /**
         * Checks if the transition command is currently running.
         * A command is considered running if it is not aborted and not complete.
         * @return true if the transition command is running, false otherwise
         */
        public boolean isRunning() {
            return !aborted() && !isComplete();
        }

        /**
         * Sets the completion status to null, indicating that the command has been aborted.
         * This method is used to mark the command as aborted.
         */
        public void setAborted() {
            setComplete.accept(null);
        }

        /**
         * Sets the completion status to true, indicating that the command has been completed.
         * This method is used to mark the command as complete.
         */
        public void setComplete() {
            setComplete.accept(true);
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
        T[] states = enumClass.getEnumConstants();
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
    public TeleOpStateHandler(T initialState) {
        this.currentGraphElement = initialState;
        this.enumClass = initialState.getDeclaringClass();
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
        return adjMatrix.get(current.ordinal(), next.ordinal()) == 1;
    }

    /**
     * Sets the current state of the teleoperated mode.
     * @param currentState the new current state to be set
     */
    public void setCurrentState(T currentState) {
        this.currentGraphElement = currentState;
    }

    /**
     * Runs a transition command to change the current state to the next state.
     * If the transition is valid, it executes the command and updates the current state.
     * If the transition is not valid, it checks if there is a pending transition and retries it after that transition is complete.
     * @param command the transition command to be executed
     * @param nextState the next state to transition to
     * @param timeout an optional timeout for the transition command
     * @param force whether to force the transition regardless of the current state
     */
    public void runTransition(TransitionCommand command, T nextState, Integer timeout, boolean force) {
        TransitionOperator transitionOperator = new TransitionOperator(command);
        T currentState = currentState();

        if ((force || currentGraphElement instanceof State) && evaluate(currentState, nextState)) {
            run(transitionOperator, nextState, timeout, currentState);
        } else if (next == null && currentGraphElement instanceof Transition transition) {
            State state = transition.next();
            if (!enumClass.isInstance(state)) {
                throw new IllegalStateException("Current state is not of expected enum type.");
            }
            T pendingNext = enumClass.cast(state);
            assert pendingNext != null;
            if (evaluate(pendingNext, nextState)) {
                next = transitionOperator;
                Scheduler.getInstance().schedule(
                        new Race(
                                new Sequential(
                                        new WaitUntil(() -> currentGraphElement instanceof State),
                                        new Instant(() -> {
                                            run(transitionOperator, nextState, timeout, currentState);
                                            next = null;
                                        })
                                ),
                                new Sequential(
                                        new WaitUntil(transitionOperator::aborted),
                                        new Instant(() -> retry(command, nextState, timeout, force))
                                )
                        )
                );
            } else {
                Log.e("TeleOpStateHandler", "Transition from " + currentState + " to " + nextState + " is not valid.");
            }
        } else if (currentGraphElement instanceof State) {
            Log.e("TeleOpStateHandler", "Transition from " + currentState + " to " + nextState + " is not valid.");
        }
    }

    /**
     * Retries a transition command if the current state is valid for the next state.
     * @param command the transition command to be executed
     * @param nextState the next state to transition to
     * @param timeout an optional timeout for the transition command
     * @param force whether to force the transition regardless of the current state
     */
    private void retry(TransitionCommand command, T nextState, Integer timeout, boolean force) {
        TransitionOperator transitionOperator = new TransitionOperator(command);
        T currentState = currentState();

        if ((force || currentGraphElement instanceof State) && evaluate(currentState, nextState)) {
            run(transitionOperator, nextState, timeout, currentState);
        }
    }

    /**
     * Runs a transition command and updates the current graph element to a new Transition.
     * It also schedules a delay to set the current state after the command is complete.
     * If a timeout is specified, it sets the maximum time to run the command.
     * @param command the transition command to be executed
     * @param nextState the next state to transition to
     * @param timeout an optional timeout for the transition command
     * @param currentState the current state before the transition
     */
    private void run(TransitionOperator command, T nextState, Integer timeout, T currentState) {
        command.run();
        currentGraphElement = new Transition(currentState, nextState, command);
        Scheduler.getInstance().schedule(
                new Race(
                        new Sequential(
                                new WaitUntil(command::isComplete),
                                new Instant(() -> setCurrentState(nextState))
                        ),
                        new Sequential(
                                new WaitUntil(command::aborted),
                                new Instant(() -> setCurrentState(currentState))
                        ),
                        new Wait(timeout != null ? timeout : 0)
                )
        );
    }

    /**
     * Runs a transition command to change the current state to the next state.
     * This method allows for an optional timeout and a force flag to override the current state.
     * @param command the transition command to be executed
     * @param nextState the next state to transition to
     * @param timeout an optional timeout for the transition command
     */
    public void runTransition(TransitionCommand command, T nextState, Integer timeout) {
        runTransition(command, nextState, timeout, force);
    }

    /**
     * Runs a transition command to change the current state to the next state.
     * This method allows for a force flag to override the current state.
     * @param command the transition command to be executed
     * @param nextState the next state to transition to
     */
    public void runTransition(TransitionCommand command, T nextState) {
        runTransition(command, nextState, null, force);
    }

    /**
     * Gets the current state of the teleoperated mode.
     * @return the current state of the teleoperated mode
     */
    public T currentState() {
        State state = currentGraphElement.getCurrentState();
        if (!enumClass.isInstance(state)) {
            throw new IllegalStateException("Current state is not of expected enum type.");
        }
        return enumClass.cast(state);
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
    public void setting(Runnable setting, List<GraphElement> dependencies) {
        if (force) {
            setting.run();
            return;
        }

        if (!dependencies.contains(currentGraphElement))
            setting.run();

        Scheduler.getInstance().schedule(
                new Sequential(
                        new WaitUntil(() -> !dependencies.contains(currentGraphElement)),
                        new Instant(setting)
                )
        );
    }

    /**
     * Executes this setting after the current transition is completed, or immediately if the current graph element isn't a transition.
     */
    public void setting(Runnable setting) {
        if (force) {
            setting.run();
            return;
        }

        if (currentGraphElement instanceof State) setting.run();
        else
            Scheduler.getInstance().schedule(
                    new Sequential(
                            new WaitUntil(() -> currentElement() instanceof State),
                            new Instant(setting)
                    )
            );
    }

    /**
     * Executes a task if the current state matches the component vector.
     * The component vector is a binary vector that indicates which components are active in the current state.
     * @param task the task to be executed
     * @param componentVector the binary vector representing the active components in the current state
     */
    public void task(Runnable task, int[] componentVector) {
        if (force) {
            task.run();
            return;
        }

        if (currentGraphElement instanceof State) {
            if (componentVector[currentState().ordinal()] == 1) task.run();
        }

        else if (currentGraphElement instanceof Transition transition)
            if (componentVector[Objects.requireNonNull(enumClass.cast(transition.next())).ordinal()] == 1)
                Scheduler.getInstance().schedule(
                        new Race(
                                new Sequential(
                                        new WaitUntil(transition::isComplete),
                                        new Instant(task)
                                ),
                                new WaitUntil(transition::isAborted)
                        )
                );
    }

    /**
     * Executes a task if the current state matches the specified state.
     * @param task the task to be executed
     * @param state the state to check against the current state
     */
    public void task(Runnable task, T state) {
        if (force) {
            task.run();
            return;
        }

        if (currentGraphElement instanceof State curState) {
            if (curState == state) task.run();
        }

        else if (currentGraphElement instanceof Transition transition)
            if (enumClass.cast(transition) == state)
                Scheduler.getInstance().schedule(
                        new Race(
                                new Sequential(
                                        new WaitUntil(transition::isComplete),
                                        new Instant(task)
                                ),
                                new WaitUntil(transition::isAborted)
                        )
                );
    }

    /**
     * Aborts the current transition if it is a Transition instance.
     * This method sets the transition command to aborted, allowing for a clean exit from the transition.
     */
    public void abortTransition() {
        if (currentGraphElement instanceof Transition transition) {
            transition.transitionCommand().setAborted();
        }
    }

    /**
     * Overrides the current transition with a new action and sets the next state.
     * This method allows for a clean override of the current transition, ensuring that the new action is executed
     * and the current state is updated to the next state.
     * @param overrideAction the action to be executed as an override
     * @param nextState the next state to transition to after the override
     */
    public void override(Runnable overrideAction, T nextState) {
        if (currentGraphElement instanceof Transition transition) {
            transition.transitionCommand.setAborted();
        }

        overrideAction.run();
        setCurrentState(nextState);
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
                return enumClass.cast(state);
            } catch (Exception ignored) {
                throw new RuntimeException();
            }
        }

        else if (currentGraphElement instanceof Transition transition) {
            try {
                return enumClass.cast(transition.next);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        return null;
    }

    public static Transition referenceTransition(State current, State next) {
        return new Transition(current, next, new TransitionOperator(t -> {}));
    }
}