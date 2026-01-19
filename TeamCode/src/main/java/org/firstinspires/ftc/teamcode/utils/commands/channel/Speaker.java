package org.firstinspires.ftc.teamcode.utils.commands.channel;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.groups.Sequential;

import java.util.function.Function;

public class Speaker<T> extends Command {
    private final Function<Channel<T>, ICommand> command;
    private Channel<T> channel;

    public Speaker(Function<Channel<T>, ICommand> command) {
        this.command = command;
    }

    public static <S> Speaker<S> sequentialCompose(ICommand command, Speaker<S> notifier) {
        return new Speaker<>(ch -> command.then(notifier));
    }

    public static  <S> Speaker<S> sequentialCompose(Speaker<S> notifier, ICommand command) {
        return new Speaker<>(ch -> notifier.then(command));
    }

    public static <S> Speaker<S> sequentialCompose(Speaker<S> notifier1, Speaker<S> notifier2) {
        return new Speaker<>(ch -> new Sequential(notifier1, notifier2));
    }

    public static <S> Speaker<S> parallelCompose(ICommand command, Speaker<S> notifier) {
        return new Speaker<>(ch -> command.with(notifier));
    }

    public static <S> Speaker<S> parallelCompose(Speaker<S> notifier, ICommand command) {
        return new Speaker<>(ch -> notifier.with(command));
    }

    public static <S> Speaker<S> parallelCompose(Speaker<S> notifier1, Speaker<S> notifier2) {
        return new Speaker<>(ch -> notifier1.with(notifier2));
    }

    public Speaker<T> subscribe(Channel<T> channel) {
        this.channel = channel;
        return this;
    }

    @Override
    public void start() {
        adoptBehavior(command.apply(channel), true);
    }
}
