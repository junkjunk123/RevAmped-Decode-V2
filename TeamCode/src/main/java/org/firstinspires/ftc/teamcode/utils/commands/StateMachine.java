package org.firstinspires.ftc.teamcode.utils.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;

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
        public final T nextState;
        public final Command command;

        protected Edge(T nextState,  Command command) {
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
        T returnVal = currentGraphElement instanceof Edge ? ((Edge) currentGraphElement).nextState : currentState;
        return returnVal == null? currentState : returnVal;
    }

    public  Command runTransition( Command transition, T newState) {
        return runTransition(transition, () -> newState);
    }

    public abstract  Command runTransition( Command transition, Supplier<T> newState);
}
