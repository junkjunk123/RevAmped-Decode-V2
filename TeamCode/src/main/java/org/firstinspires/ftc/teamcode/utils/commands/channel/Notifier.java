package org.firstinspires.ftc.teamcode.utils.commands.channel;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.groups.Sequential;

import java.util.function.Function;

public class Notifier extends Speaker<Object> {
    public Notifier(Function<Channel<Object>, ICommand> command) {
        super(command);
    }

    public static Notifier sequentialCompose(ICommand command, Notifier notifier) {
        return new Notifier(ch -> command.then(notifier));
    }

    public static Notifier sequentialCompose(Notifier notifier, ICommand command) {
        return new Notifier(ch -> notifier.then(command));
    }

    public static Notifier sequentialCompose(Notifier notifier1, Notifier notifier2) {
        return new Notifier(ch -> new Sequential(notifier1, notifier2));
    }

    public static Notifier parallelCompose(ICommand command, Notifier notifier) {
        return new Notifier(ch -> command.with(notifier));
    }

    public static Notifier parallelCompose(Notifier notifier, ICommand command) {
        return new Notifier(ch -> notifier.with(command));
    }

    public static Notifier parallelCompose(Notifier notifier1, Notifier notifier2) {
        return new Notifier(ch -> notifier1.with(notifier2));
    }
}
