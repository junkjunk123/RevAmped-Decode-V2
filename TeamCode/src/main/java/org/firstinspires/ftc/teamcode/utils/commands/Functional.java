package org.firstinspires.ftc.teamcode.utils.commands;

import com.pedropathing.ivy.Command;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class Functional extends Command {
    public Functional(Runnable start, Runnable execute, BooleanSupplier isDone) {
        setStart(start);
        setExecute(execute);
        setDone(isDone);
    }

    public Functional(Runnable start, Runnable execute, BooleanSupplier isDone, Consumer<Boolean> end) {
        this(start, execute, isDone);
        setEnd(end);
    }
}
