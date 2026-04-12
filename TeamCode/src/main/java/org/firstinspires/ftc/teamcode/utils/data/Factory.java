package org.firstinspires.ftc.teamcode.utils.data;
import java.util.ArrayList;
import java.util.function.Function;

public class Factory<T> {
    private final Function<T, T> operator;
    private final ArrayList<T> creations = new ArrayList<>();

    public Factory() {
        this(i -> i);
    }

    public Factory(Function<T, T> operator) {
        this.operator = operator;
    }

    public T output(T input) {
        return operator.apply(input);
    }

    public void create(T input) {
        creations.add(output(input));
    }

    public ArrayList<T> compose() {
        return creations;
    }
}
