package org.firstinspires.ftc.teamcode.utils.data;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class AtomicReadOnce<T> {
    private final AtomicReference<T> ref = new AtomicReference<>();
    private final Supplier<T> supplier;

    public AtomicReadOnce(Supplier<T> supplier) {
        this.supplier = supplier;
    }

    public T read() {
        T value = ref.get();
        if (value == null) {
            T computed = supplier.get();
            if (ref.compareAndSet(null, computed)) {
                return computed;
            } else {
                return ref.get();
            }
        }
        return value;
    }

    public boolean hasBeenRead() {
        return ref.get() == null;
    }
}
