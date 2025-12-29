package org.firstinspires.ftc.teamcode.utils;

import java.util.concurrent.atomic.AtomicReference;

public class AtomicWriteOnce<T> {
    private final AtomicReference<T> ref = new AtomicReference<>();

    /**
     * Attempts to set the value. Only succeeds if value is not already set.
     *
     * @param value the value to set
     * @return true if successful, false if already set
     */
    public boolean set(T value) {
        return ref.compareAndSet(null, value);
    }

    /**
     * Gets the value. Returns null if not yet set.
     */
    public T get() {
        return ref.get();
    }
}
