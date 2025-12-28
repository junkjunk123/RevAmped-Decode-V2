package org.firstinspires.ftc.teamcode.utils.commands;

import com.pedropathing.ivy.ICommand;

public abstract class StateMachine<T extends Enum<T>> {
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
        public final T nextState;
        public final ICommand command;

        protected Edge(T nextState, ICommand command) {
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

    public abstract ICommand runTransition(ICommand transition, T newState);
}
