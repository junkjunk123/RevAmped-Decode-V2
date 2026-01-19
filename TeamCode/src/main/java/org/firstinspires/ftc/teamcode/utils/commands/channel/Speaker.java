package org.firstinspires.ftc.teamcode.utils.commands.channel;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;
import java.util.function.Function;

public class Speaker<T> extends Command {
    private final Function<Channel<T>, ICommand> command;
    private Channel<T> channel;

    public Speaker(Function<Channel<T>, ICommand> command) {
        this.command = command;
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
