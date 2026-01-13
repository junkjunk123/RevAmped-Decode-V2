package org.firstinspires.ftc.teamcode.utils.commands.channel;
import java.util.ArrayDeque;
import java.util.Deque;

public class Channel<T> {
    private final Deque<T> stream = new ArrayDeque<>();
    private int capacity = -1;

    void send(T channelMessage) {
        stream.addLast(channelMessage);
        if (capacity != -1) {
            while (stream.size() > capacity) {
                stream.pollFirst();
            }
        }
    }

    T receive() {
        return stream.poll();
    }

    T receiveLast() {
        return stream.pollLast();
    }

    public boolean hasMessages() {
        return !stream.isEmpty();
    }

    public int size() {
        return stream.size();
    }

    Channel() {}
    Channel(int capacity) {this.capacity = capacity;}
}
