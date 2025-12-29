package org.firstinspires.ftc.teamcode.utils.commands;

import com.pedropathing.ivy.ICommand;

import java.util.function.Supplier;

public abstract class StateMachine<T> {
    protected GraphElement currentGraphElement;
    private T currentState;

    protected StateMachine(T initialState) {
        setCurrentState(initialState);
    }

    protected sealed abstract class GraphElement permits Node, Edge {
    }

    protected non-sealed class Node extends GraphElement {
        public final T state;

        protected Node(T state) {
            this.state = state;
        }
    }

    protected non-sealed class Edge extends GraphElement {
        public final Supplier<T> nextState;
        public final ICommand command;

        protected Edge(Supplier<T> nextState, ICommand command) {
            this.nextState = nextState;
            this.command = command;
        }
    }

    public T getCurrentState() {
        return currentState;
    }

    public void setCurrentState(T state) {
        currentGraphElement = new Node(state);
        currentState = state;
    }

    public T getPendingState() {
        return currentGraphElement instanceof Edge ? ((Edge) currentGraphElement).nextState.get() : currentState;
    }

    public ICommand runTransition(ICommand transition, T newState) {
        return runTransition(transition, () -> newState);
    }

    public abstract ICommand runTransition(ICommand transition, Supplier<T> newState);
}
