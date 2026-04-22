package org.firstinspires.ftc.teamcode.utils.data;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

public class ListMap<K, V> {
    public record Entry<K, V>(K key, V val) {}

    private final List<Entry<K, V>> pairs = new ArrayList<>();
    private final HashMap<K, V> map = new LinkedHashMap<>();

    public ListMap<K, V> add(K key, V val) {
        map.put(key, val);
        pairs.add(new Entry<>(key, val));
        return this;
    }

    public Entry<K, V> get(int i) {
        return pairs.get(i);
    }

    public int size() {
        return pairs.size();
    }

    public boolean isEmpty() {
        return pairs.isEmpty();
    }

    public V get(K key) {
        return map.get(key);
    }
}
