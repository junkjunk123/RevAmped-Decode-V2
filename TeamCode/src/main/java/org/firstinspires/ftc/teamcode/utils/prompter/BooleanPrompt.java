package org.firstinspires.ftc.teamcode.utils.prompter;

public class BooleanPrompt extends Prompt<Boolean> {
    private final String header;
    private boolean selectedValue;

    public BooleanPrompt(String header, boolean defaultValue) {
        this.header = header;
        this.selectedValue = defaultValue;
    }

    @Override
    public Boolean process() {
        addLine(header);
        addLine("");
        addLine("Current Value: " + (selectedValue ? "Yes" : "No"));

        if (anyJustPressed(
                input.dpad_up,
                input.dpad_down,
                input.dpad_left,
                input.dpad_right
        )) {
            selectedValue = !selectedValue;
        }

        if (input.right_bumper.isRisingEdge()) {
            return selectedValue;
        }

        return null;
    }
}
