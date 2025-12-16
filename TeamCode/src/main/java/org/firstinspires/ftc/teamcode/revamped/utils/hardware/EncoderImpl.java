package org.firstinspires.ftc.teamcode.revamped.utils.hardware;

import org.firstinspires.ftc.teamcode.revamped.math.calc.Differentiator;

public class EncoderImpl implements Encoder {
    private final Encoder rawEncoder;
    private Integer currentPos;
    private final Differentiator velocity;
    private int encoderBase = 0;

    public EncoderImpl(Encoder rawEncoder) {
        this.rawEncoder = rawEncoder;
        velocity = new Differentiator(rawEncoder);
    }

    @Override
    public int getPosition() {
        if (currentPos == null)
            currentPos = rawEncoder.getPosition() - encoderBase;
        return currentPos;
    }

    @Override
    public double getVelocity() {
        return velocity.calculate();
    }

    public void reset() {
        velocity.reset();
    }

    public void update() {
        currentPos = null;
        velocity.update();
    }

    public void resetPosition() {
        resetPosition(0);
    }

    public void resetPosition(int val) {
        encoderBase = rawEncoder.getPosition() - val;
    }
}
