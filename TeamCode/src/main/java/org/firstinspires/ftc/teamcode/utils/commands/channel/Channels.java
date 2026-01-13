package org.firstinspires.ftc.teamcode.utils.commands.channel;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import java.util.function.Supplier;

public class Channels {
    public static <T> Channel<T> oneshot() {
        return new Channel<T>(1);
    }

    public static <T> Channel<T> stream() {
        return new Channel<T>();
    }

    public static <T> ICommand send(Channel<T> channel, Supplier<T> message) {
        return new Instant(() -> {
            if (channel == null) return;
            channel.send(message.get());
        });
    }

    public static <T> T receive(Channel<T> channel) {
        if (!channel.hasMessages()) return null;
        @SuppressWarnings("unchecked")
        T channelMessage = (T) channel.receive();
        return channelMessage;
    }

    public static <T> boolean hasMessages(Channel<T> channel) {
        return channel.hasMessages();
    }

    public static <T> T receiveLast(Channel<T> channel) {
        @SuppressWarnings("unchecked")
        T channelMessage = (T) channel.receiveLast();
        return channelMessage;
    }

    public static int size(Channel<?> channel) {
        return channel.size();
    }

    public static <T> T signal() {
        return null;
    }
}
