package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class BooleanPeriod implements BooleanSupplier {
    private final ElapsedTime timer = new ElapsedTime();
    private boolean reading;
    private double period;
    private final BooleanSupplier supplier;
    private boolean lastValue;

    public BooleanPeriod(BooleanSupplier supplier, double periodMs) {
        this.period = periodMs;
        this.supplier = supplier;
    }

    public void setPeriod(double period) {
        this.period = period;
    }

    public void start() {
        reading = true;
    }

    public void update() {
        if (!reading) return;
        lastValue = supplier.getAsBoolean();
        if (!lastValue) timer.reset();
    }

    public boolean isLastValue() {
        return lastValue;
    }

    @Override
    public boolean getAsBoolean() {
        return timer.milliseconds() >= period;
    }
}
