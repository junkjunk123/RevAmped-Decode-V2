package org.firstinspires.ftc.teamcode.utils.data;

import androidx.annotation.NonNull;

import java.util.Iterator;
import java.util.NoSuchElementException;

public class FiniteSizedQueue<Item> implements Iterable<Item> {
    private Node last;
    private final int capacity;
    private int n;

    public FiniteSizedQueue(int capacity) {
        last = null;
        this.capacity = capacity;
    }

    @NonNull
    public Iterator<Item> iterator() {
        return new LinkedIterator();
    }

    private class Node {
        Node next;
        Item item;

        Node(Item item, Node next) {
            this.item = item;
            this.next = next;
        }

        public Node() {}

        public Node(Item item) {
            this.item = item;
        }
    }

    /**
     * Returns a string representation of this queue.
     *
     * @return the sequence of items in FIFO order, separated by spaces
     */
    @NonNull
    public String toString() {
        StringBuilder s = new StringBuilder();
        for (Item item : this) {
            s.append(item);
            s.append(' ');
        }
        return s.toString();
    }

    public boolean isEmpty() {
        return last == null;
    }

    /**
     * Returns the item least recently added to this queue.
     *
     * @return the item least recently added to this queue
     * @throws NoSuchElementException if this queue is empty
     */
    public Item peek() {
        if (isEmpty()) throw new NoSuchElementException("Queue underflow");
        return last.next.item;
    }

    public void enqueue(Item item) {
        Node newNode = new Node(item);

        if (isEmpty()) {
            last = newNode;
            last.next = last;
            if (n < capacity) n++;
            else dequeue();
            return;
        }

        newNode.next = last.next;
        last.next = newNode;
        last = newNode;
        if (n < capacity) n++;
        else dequeue();
    }

    public Item dequeue() {
        if (isEmpty())
            throw new NoSuchElementException("cannot delete from empty queue");
        Node first = last.next;
        if (first == last)
            last = null;
        else
            last.next = first.next;
        n--;
        return first.item;
    }

    private class LinkedIterator implements Iterator<Item> {
        private Node current;
        private boolean circulated = false;

        public LinkedIterator() {
            current = last.next;
        }

        public boolean hasNext() {
            return current != null && !circulated;
        }

        public Item next() {
            if (!hasNext()) throw new NoSuchElementException();
            if (current == last) circulated = true;
            Item item = current.item;
            current = current.next;
            return item;
        }
    }
}