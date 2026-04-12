package org.firstinspires.ftc.teamcode.utils.data;

import java.util.AbstractMap;
import java.util.HashMap;
import java.util.function.Supplier;

public class MapBuilder<T, V, K extends AbstractMap<T, V>> {
    private final K map;

    private MapBuilder(K map) {
        this.map = map;
    }

    public static <T, V, K extends AbstractMap<T, V>> MapBuilder<T, V, K> create(Supplier<K> ref) {
        return new MapBuilder<>(ref.get());
    }

    public static <T, V> MapBuilder<T, V, HashMap<T, V>> create() {
        return new MapBuilder<>(new HashMap<>());
    }

    public MapBuilder<T, V, K> add(T key, V value) {
        map.put(key, value);
        return this;
    }

    public K getMap() {
        return map;
    }
}
