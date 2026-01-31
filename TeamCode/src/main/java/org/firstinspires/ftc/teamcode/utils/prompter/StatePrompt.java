package org.firstinspires.ftc.teamcode.utils.prompter;

import java.util.function.Function;

public class StatePrompt<T extends Enum<T>> extends OptionPrompt<T> {
    public StatePrompt(String header, Class<T> classReference) {
        super(header, classReference.getEnumConstants());
    }

    public StatePrompt(String header, Function<T, String> optionToString, Class<T> classReference) {
        super(header, optionToString, classReference.getEnumConstants());
    }
}
