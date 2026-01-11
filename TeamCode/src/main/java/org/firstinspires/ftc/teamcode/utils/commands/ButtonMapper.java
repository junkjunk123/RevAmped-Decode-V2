package org.firstinspires.ftc.teamcode.utils.commands;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.utils.BooleanSwitch;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ButtonMapper {
    private final HashMap<BooleanSwitch, Supplier<ICommand>> buttonMap;

    public ButtonMapper() {
        buttonMap = new HashMap<>();
    }

    public ButtonMapper put(BooleanSwitch b, Supplier<ICommand> c) {
        buttonMap.put(b, c);
        return this;
    }

    public ButtonMapper put(BooleanSupplier b, Supplier<ICommand> c) {
        return put(new BooleanSwitch(b), c);
    }

    public void update() {
        for (Map.Entry<BooleanSwitch, Supplier<ICommand>> e : buttonMap.entrySet()) {
            e.getKey().update();

            if (e.getKey().isTrue())
                Scheduler.getInstance().schedule(e.getValue().get());
        }
    }
}
