package org.firstinspires.ftc.teamcode.utils.prompter;

import java.util.function.Function;

public class OptionPrompt<T> extends Prompt<T> {
    private final String header;
    private final T[] options;
    private int selectedOptionIndex = 0;
    private final Function<T, String> optionToString;

    @SafeVarargs
    public OptionPrompt(String header, T... options) {
        this.header = header;
        this.options = options;
        this.optionToString = Object::toString;
    }

    @SafeVarargs
    public OptionPrompt(String header, Function<T, String> optionToString, T... options) {
        this.header = header;
        this.options = options;
        this.optionToString = optionToString;
    }

    @Override
    public T process() {
        addLine(header);
        addLine("");

        for (int i = 0; i < options.length; i++) {
            if (i == selectedOptionIndex) {
                addLine((i + 1) + ") " + optionToString.apply(options[i]) + " <");
            } else {
                addLine((i + 1) + ") " + optionToString.apply(options[i]));
            }
        }

        if (input.dpad_up.isRisingEdge()) {
            selectedOptionIndex = (selectedOptionIndex - 1 + options.length) % options.length;
        } else if (input.dpad_down.isRisingEdge()) {
            selectedOptionIndex = (selectedOptionIndex + 1) % options.length;
        }

        if (input.right_bumper.isRisingEdge()) {
            return options[selectedOptionIndex];
        }

        return null;
    }
}
