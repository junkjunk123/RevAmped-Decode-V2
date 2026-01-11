package org.firstinspires.ftc.teamcode.utils.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;

import java.util.function.BooleanSupplier;

public class Conditional extends Command {
    private final BooleanSupplier decider;
    private final ICommand option1;
    private final ICommand option2;

    public Conditional(BooleanSupplier decider, ICommand ifTrue, ICommand ifFalse) {
        this.decider = decider;
        this.option1 = ifTrue;
        this.option2 = ifFalse;
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
