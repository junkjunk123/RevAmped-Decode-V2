package org.firstinspires.ftc.teamcode.utils.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;

import java.util.LinkedHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

public class Option extends Command {
    private final LinkedHashMap<BooleanSupplier, ICommand> commands;

    public Option(LinkedHashMap<BooleanSupplier, ICommand> commands) {
        this.commands = commands;
    }

    @Override
    public void start() {
        AtomicBoolean foundOne = new AtomicBoolean(false);
        commands.forEach((u, v) -> {
            if (!foundOne.get()) {
                if (u.getAsBoolean()) {
                    foundOne.set(true);
                    v.start();
                    setExecute(v::execute);
                    setEnd(v::end);
                    setDone(v::done);
                    setInterruptibility(v.getInterruptibility());
                    if (v.getRequirements() != null)
                        setRequirements(v.getRequirements().toArray());
                }
            }
        });
    }
}
