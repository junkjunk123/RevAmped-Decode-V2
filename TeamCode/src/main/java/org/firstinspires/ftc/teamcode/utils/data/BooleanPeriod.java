package org.firstinspires.ftc.teamcode.utils.data;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class BooleanPeriod implements BooleanSupplier {
    private final ElapsedTime timer = new ElapsedTime();
    private boolean reading;
    private double period;
    private final BooleanSupplier supplier;
    private boolean lastValue;
    private final int numMisfires;
    private int misfired;

    public BooleanPeriod(BooleanSupplier supplier, double periodMs, int numMisfires) {
        this.period = periodMs;
        this.supplier = supplier;
        this.numMisfires = numMisfires;
    }

    public BooleanPeriod(BooleanSupplier supplier, double periodMs) {
        this(supplier, periodMs, 0);
    }

    public void setPeriod(double period) {
        this.period = period;
    }

    public void start() {
        reading = true;
        reset();
    }

    public void stop() {
        reading = false;
    }

    public void reset() {
        timer.reset();
        misfired = 0;
    }

    public void update() {
        if (!reading) return;
        lastValue = supplier.getAsBoolean();

        if (!lastValue) {
            misfired++;
            if (misfired > numMisfires) reset();
        }
    }

    public boolean isLastValue() {
        return lastValue;
    }

    @Override
    public boolean getAsBoolean() {
        return timer.milliseconds() >= period;
    }
}
