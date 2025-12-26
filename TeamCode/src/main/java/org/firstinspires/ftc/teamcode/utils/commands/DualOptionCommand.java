package org.firstinspires.ftc.teamcode.utils.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;

import java.util.function.BooleanSupplier;

public class DualOptionCommand extends Command {
    private final BooleanSupplier decider;
    private final ICommand option1;
    private final ICommand option2;

    public DualOptionCommand(BooleanSupplier decider, ICommand option1, ICommand option2) {
        this.decider = decider;
        this.option1 = option1;
        this.option2 = option2;
    }

    @Override
    public void start() {
        if (decider.getAsBoolean()) {
            copy(option1);
            return;
        }

        copy(option2);
    }

    private void copy(ICommand v) {
        v.start();
        setExecute(v::execute);
        setEnd(v::end);
        setDone(v::done);
        setInterruptibility(v.getInterruptibility());
        if (v.getRequirements() != null)
            setRequirements(v.getRequirements().toArray());
    }
}
