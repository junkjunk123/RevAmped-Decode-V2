package org.firstinspires.ftc.teamcode.utils.math;

public class Z3Element {
    private final int val;

    public Z3Element(int element) {
        while (element < 0) element += 3;
        val = element % 3;
    }

    public int getVal() {
        return val;
    }

    public Z3Element plus(int i) {
        return new Z3Element(val + i);
    }

    public Z3Element minus(int i) {
        return new Z3Element(val - i);
    }

    public Z3Element times(int i) {
        return new Z3Element(val * i);
    }
}
