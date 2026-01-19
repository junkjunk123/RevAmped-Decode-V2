package org.firstinspires.ftc.teamcode.utils.commands.channel;
import com.pedropathing.ivy.ICommand;
import java.util.function.Function;

public class Notifier extends Speaker<Object> {
    public Notifier(Function<Channel<Object>, ICommand> command) {
        super(command);
    }
}
