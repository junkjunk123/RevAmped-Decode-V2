package org.firstinspires.ftc.teamcode.utils.commands;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.utils.AtomicReadOnce;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.IntSupplier;

public class RevAmpedCommands {
    public static Command loop(Command command, IntSupplier iterations) {
        AtomicInteger integer = new AtomicInteger();
        AtomicReadOnce<Integer> iterationsReadOnce = new AtomicReadOnce<>(iterations::getAsInt);
        return Groups.loop(command.with(instant(integer::getAndIncrement))).setDone(() -> integer.get() < iterationsReadOnce.read());
    }
}
