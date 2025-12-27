package org.firstinspires.ftc.teamcode.utils.prompter;

public class ValuePrompt extends Prompt<Double> {
    private final String header;
    private final double minValue;
    private final double maxValue;
    private double increment;
    private double selectedValue;
    private final double incrementIncrement;
    private boolean isInteger = false;

    public ValuePrompt(String header) {
        this(header, Double.MIN_VALUE, Double.MAX_VALUE, 0, 1);
    }

    public ValuePrompt(String header, double defaultValue) {
        this(header, Double.MIN_VALUE, Double.MAX_VALUE, defaultValue, 1);
    }

    public ValuePrompt(String header, double defaultValue, double increment) {
        this(header, Double.MIN_VALUE, Double.MAX_VALUE, defaultValue, increment);
    }

    public ValuePrompt(String header, double minValue, double maxValue, double defaultValue, double increment) {
        this.header = header;
        this.minValue = minValue;
        this.maxValue = maxValue;
        this.increment = increment;
        this.selectedValue = defaultValue;
        incrementIncrement = 0;

        if (isInteger(maxValue, minValue, defaultValue, increment, incrementIncrement)) isInteger = true;
    }

    public ValuePrompt(String header, double minValue, double maxValue, double defaultValue, double increment, double incrementIncrement) {
        this.header = header;
        this.minValue = minValue;
        this.maxValue = maxValue;
        this.increment = increment;
        this.selectedValue = defaultValue;
        this.incrementIncrement = incrementIncrement;

        if (isInteger(maxValue, minValue, defaultValue, increment, incrementIncrement)) isInteger = true;
    }

    private boolean isInteger(double... a) {
        for (double i : a) if (Math.round(i) != i) return false;
        return true;
    }

    @Override
    public Double process() {
        addLine(header);
        addLine("");

        if (!isInteger) addLine("< " + selectedValue + " >");
        else addLine("< " + (int) selectedValue + " >");
        addData("Increment",increment);

        if (input.dpad_up.isRisingEdge()) {
            selectedValue = Math.min(maxValue, selectedValue + increment);
        } else if (input.dpad_down.isRisingEdge()) {
            selectedValue = Math.max(minValue, selectedValue - increment);
        } else if (input.dpad_left.isRisingEdge()) {
            if (incrementIncrement != 0) increment -= incrementIncrement;
            else increment /= 2;
        } else if (input.dpad_right.isRisingEdge()) {
            if (incrementIncrement != 0) increment += incrementIncrement;
            else increment *= 2;
        }

        if (input.right_bumper.isRisingEdge()) {
            return selectedValue;
        }

        return null;
    }
}
