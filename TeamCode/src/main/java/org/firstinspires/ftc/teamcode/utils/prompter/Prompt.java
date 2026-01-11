package org.firstinspires.ftc.teamcode.utils.prompter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.BooleanSwitch;
import org.firstinspires.ftc.teamcode.utils.GamepadEx;

public abstract class Prompt<T> {
    protected GamepadEx input;
    private Telemetry telemetry;

    public abstract T process();

    /**
     * Injects dependencies into the prompt.
     * This method is package-private to allow Prompter to configure the prompt
     * without exposing these dependencies publicly.
     */
    void configure(GamepadEx input, Telemetry telemetry) {
        this.input = input;
        this.telemetry = telemetry;
    }

    // Helper functions

    /**
     * Checks if any of the specified buttons were just pressed.
     */
    protected boolean anyJustPressed(BooleanSwitch... buttons) {
        for (BooleanSwitch button : buttons) {
            if (button.isRisingEdge()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Adds a line to the Telemetry.
     */
    protected void addLine(String lineCaption) {
        telemetry.addLine(lineCaption);
    }

    /**
     * Adds data to the Telemetry.
     */
    protected void addData(String caption, Object value) {
        telemetry.addData(caption, value);
    }
}
