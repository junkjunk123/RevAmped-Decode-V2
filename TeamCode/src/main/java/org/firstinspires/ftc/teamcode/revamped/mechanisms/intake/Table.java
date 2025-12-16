package org.firstinspires.ftc.teamcode.revamped.mechanisms.intake;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.revamped.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.EncoderImpl;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.HwServo;
import dev.frozenmilk.dairy.mercurial.continuations.Continuation;
import dev.frozenmilk.dairy.mercurial.continuations.Continuations;
import dev.frozenmilk.dairy.mercurial.continuations.Scheduler;

public class Table extends HwServo {
    public enum RelativeState {
        BALL1,
        BALL2,
        BALL3;

        public float[] getShootStates() {
            switch (this) {
                case BALL1 -> {
                    return new float[] {Table.BALL2, Table.BALL3, Table.BALL3_END};
                }
                case BALL2 -> {
                    return new float[] {Table.BALL3, Table.BALL1_REV2, Table.BALL2_END};
                }
                case BALL3 -> {
                    return new float[] {Table.BALL1_REV2, Table.BALL2_REV2, Table.BALL3_END};
                }
            }

            return null;
        }
    }

    public static float BALL1;
    public static float BALL2;
    public static float BALL3;
    public static float BALL1_END;
    public static float BALL2_END;
    public static float BALL3_END;
    public static float BALL1_REV2;
    public static float BALL2_REV2;
    public static float FULL_REVOLUTION;

    private RelativeState initialShootState = RelativeState.BALL2;
    private final EncoderImpl encoder;
    private final Continuation reached;
    private boolean hasReached;

    /**
     * @param hwMap hardwareMap
     * @param id    the ID of the servo as configured
     */
    public Table(HardwareMap hwMap, String id, Encoder rawEncoder) {
        super(hwMap, id);
        this.encoder = new EncoderImpl(rawEncoder);
        reached = Continuations.sequence(
                Continuations.parallel(
                    Continuations.waitSeconds(0.25),
                    Continuations.waitUntil(() -> encoder.getVelocity() < 10)
                ),
                Continuations.exec(() -> hasReached = true)
        ).close();
    }

    public boolean reached() {
        return hasReached;
    }

    public void one() {
        setPosition(BALL1);
        initialShootState = RelativeState.BALL1;
    }

    public void two() {
        setPosition(BALL2);
        initialShootState = RelativeState.BALL2;
    }

    public void three() {
        setPosition(BALL3);
        initialShootState = RelativeState.BALL3;
    }

    public void reset() {
        two();
    }

    public RelativeState getState() {
        return initialShootState;
    }

    public static void setValues(float BALL_1, float BALL_2, float BALL_1_END) {
        BALL1 = BALL_1;
        BALL2 = BALL_2;
        float diff = BALL_1 - BALL_2;
        BALL3 = diff + BALL2;
        FULL_REVOLUTION = diff * 3;
        BALL1_REV2 = BALL_1 + FULL_REVOLUTION;
        BALL2_REV2 = BALL_2 + FULL_REVOLUTION;
        BALL1_END = BALL_1_END;
        BALL2_END = BALL_1_END + diff;
        BALL3_END = BALL2_END + diff;
    }

    @Override
    public boolean setPosition(double pos) {
        if (super.setPosition(pos)) {
            encoder.reset();
            Scheduler.currentScheduler().schedule(reached);
            return true;
        }

        return false;
    }

    public void update() {
        encoder.update();
    }
}
