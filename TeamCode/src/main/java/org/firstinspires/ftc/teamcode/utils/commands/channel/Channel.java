package org.firstinspires.ftc.teamcode.utils.commands.channel;
import com.pedropathing.ivy.commands.WaitUntil;

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

    public Deque<T> getStream() {
        return stream;
    }

    T receive() {
        return stream.poll();
    }

    T receiveLast() {
        return stream.pollLast();
    }

    public void clear() {
        stream.clear();
    }

    public boolean hasMessages() {
        return !stream.isEmpty();
    }

    public int size() {
        return stream.size();
    }

    public WaitUntil listen() {
        return new WaitUntil(this::hasMessages);
    }

    public WaitUntil listenAndClear() {
        return new WaitUntil(() -> {
            boolean hasMessages = this.hasMessages();
            if (hasMessages) {
                this.clear();
            }
            return hasMessages;
        });
    }

    Channel() {}
    Channel(int capacity) {this.capacity = capacity;}
}
