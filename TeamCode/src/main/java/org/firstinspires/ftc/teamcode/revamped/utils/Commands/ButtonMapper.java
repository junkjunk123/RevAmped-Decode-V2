package org.firstinspires.ftc.teamcode.revamped.utils.Commands;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.revamped.utils.BooleanSwitch;
import java.util.HashMap;
import java.util.Map;

public class ButtonMapper {
    private final HashMap<BooleanSwitch, ICommand> buttonMap;

    public ButtonMapper() {
        buttonMap = new HashMap<>();
    }

    public ButtonMapper put(BooleanSwitch b, ICommand c) {
        buttonMap.put(b, c);
        return this;
    }

    public void update() {
        for (Map.Entry<BooleanSwitch, ICommand> e : buttonMap.entrySet()) {
            e.getKey().update();

            if (e.getKey().isTrue())
                Scheduler.getInstance().schedule(e.getValue());
        }
    }
}
