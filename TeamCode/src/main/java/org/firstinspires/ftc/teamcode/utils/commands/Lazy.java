package org.firstinspires.ftc.teamcode.utils.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;

import java.util.function.Supplier;

public class Lazy extends Command {
    private final Supplier<ICommand> supplier;

    public Lazy(Supplier<ICommand> supplier) {
        this.supplier = supplier;
    }

    @Override
    public void start() {
        copy(supplier.get());
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
