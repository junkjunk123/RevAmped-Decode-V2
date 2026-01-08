package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;

public class GamepadEx {
    public final BooleanSwitch b;
    public final BooleanSwitch a;
    public final BooleanSwitch x;
    public final BooleanSwitch y;
    public final BooleanSwitch left_bumper;
    public final BooleanSwitch right_bumper;
    public final BooleanSwitch start;
    public final BooleanSwitch back;
    public final BooleanSwitch dpad_left;
    public final BooleanSwitch dpad_up;
    public final BooleanSwitch dpad_down;
    public final BooleanSwitch dpad_right;
    public final BooleanSwitch left_stick_button;
    public final BooleanSwitch right_stick_button;
    public final BooleanSwitch touchpad;

    public Set<BooleanSwitch> buttons;

    public final FloatSupplier right_trigger;
    public final FloatSupplier left_trigger;
    public final FloatSupplier right_stick_y;
    public final FloatSupplier right_stick_x;
    public final FloatSupplier left_stick_y;
    public final FloatSupplier left_stick_x;

    public BooleanSwitch right_trigger_button;
    public BooleanSwitch left_trigger_button;

    public GamepadEx(Gamepad gamepad) {
        a = new BooleanSwitch(() -> gamepad.a);
        b = new BooleanSwitch(() -> gamepad.b);
        x = new BooleanSwitch(() -> gamepad.x);
        y = new BooleanSwitch(() -> gamepad.y);
        left_bumper = new BooleanSwitch(() -> gamepad.left_bumper);
        right_bumper = new BooleanSwitch(() -> gamepad.right_bumper);
        start = new BooleanSwitch(() -> gamepad.start);
        back = new BooleanSwitch(() -> gamepad.back);
        dpad_left = new BooleanSwitch(() -> gamepad.dpad_left);
        dpad_right = new BooleanSwitch(() -> gamepad.dpad_right);
        dpad_down = new BooleanSwitch(() -> gamepad.dpad_down);
        dpad_up = new BooleanSwitch(() -> gamepad.dpad_up);
        left_stick_button = new BooleanSwitch(() -> gamepad.left_stick_button);
        right_stick_button = new BooleanSwitch(() -> gamepad.right_stick_button);
        touchpad = new BooleanSwitch(() -> gamepad.touchpad);
        right_trigger = new FloatSupplier(() -> gamepad.right_trigger);
        left_trigger = new FloatSupplier(() -> gamepad.left_trigger);
        right_stick_x = new FloatSupplier(() -> gamepad.right_stick_x);
        left_stick_x = new FloatSupplier(() -> gamepad.left_stick_x);
        right_stick_y = new FloatSupplier(() -> gamepad.right_stick_y);
        left_stick_y = new FloatSupplier(() -> gamepad.left_stick_y);
        buttons = new HashSet<>(List.of(
                a, b, x, y, left_bumper, right_bumper, start, back, dpad_left, dpad_right, dpad_down, dpad_up, left_stick_button,
                right_stick_button, touchpad
        ));
    }

    public void apply(Function<BooleanSwitch, BooleanSwitch> function) {
        buttons = buttons.stream().map(function).collect(Collectors.toSet());
    }

    public void left_trigger_button(Function<FloatSupplier, BooleanSwitch> f) {
        left_trigger_button = f.apply(left_trigger);
        buttons.add(left_trigger_button);
    }

    public void right_trigger_button(Function<FloatSupplier, BooleanSwitch> f) {
        right_trigger_button = f.apply(right_trigger);
        buttons.add(right_trigger_button);
    }

    public void update() {
        for (BooleanSwitch button : buttons) button.update();
    }
}
