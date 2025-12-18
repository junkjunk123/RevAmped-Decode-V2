package org.firstinspires.ftc.teamcode.revamped.utils;

import java.util.function.BooleanSupplier;

public final class BooleanSwitch {
    private boolean state;
    private boolean previousState;
    private boolean toggled;
    private final BooleanSupplier decider;

    public BooleanSwitch(BooleanSupplier decider) {
        this.decider = decider;
    }

    /** Advance the state by one tick */
    public void update() {
        previousState = state;
        state = decider.getAsBoolean();

        // toggle on rising edge
        if (isRisingEdge()) {
            toggled = !toggled;
        }
    }

    public boolean isTrue() {
        return state;
    }

    public boolean isRisingEdge() {
        return state && !previousState;
    }

    public boolean isFallingEdge() {
        return !state && previousState;
    }

    public boolean isToggled() {
        return toggled;
    }

    public BooleanSupplier risingEdge() {
        return this::isRisingEdge;
    }

    public BooleanSupplier fallingEdge() {
        return this::isFallingEdge;
    }

    public BooleanSupplier toggled() {
        return this::isToggled;
    }

    public BooleanSwitch and(BooleanSwitch other) {
        return new BooleanSwitch(() -> this.state && other.state);
    }

    public BooleanSwitch or(BooleanSwitch other) {
        return new BooleanSwitch(() -> this.state || other.state);
    }

    public BooleanSwitch not() {
        return new BooleanSwitch(() -> !this.state);
    }
}