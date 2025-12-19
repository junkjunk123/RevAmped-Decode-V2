package org.firstinspires.ftc.teamcode.revamped.mechanisms.intake;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.revamped.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.EncoderImpl;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.HwServo;


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

        public double target() {
            switch (this) {
                case BALL1 -> {
                    return Table.BALL1;
                }
                case BALL2 -> {
                    return Table.BALL2;
                }
                default -> {
                    return Table.BALL3;
                }
            }
        }

        public RelativeState next() {
            return values()[(ordinal() + 1) % values().length];
        }

        public RelativeState previous() {
            return values()[(ordinal() + 2) % values().length];
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
    public static float SECONDS_PER_ROTATION = 1;

    private ICommand hasReached;

    private RelativeState state = RelativeState.BALL2;
    private final EncoderImpl encoder;
    private boolean reached;

    public Table(HardwareMap hwMap, Encoder rawEncoder) {
        super(hwMap, "tableServo");
        this.encoder = new EncoderImpl(rawEncoder);
    }

    public boolean reached() {
        return reached;
    }

    public ICommand one() {
        return setRelativeState(RelativeState.BALL1);
    }

    public ICommand two() {
        return setRelativeState(RelativeState.BALL2);
    }

    public ICommand three() {
        return setRelativeState(RelativeState.BALL3);
    }

    public void reset() {
        two();
    }

    public ICommand setState(int state) {
        return setRelativeState(RelativeState.values()[state]);
    }

    public ICommand setRelativeState(RelativeState relativeState) {
        return new Sequential(
                new Instant(() -> {
                    state = relativeState;
                    setPosition(relativeState.target());
                }),
                new Wait(250),
                new WaitUntil(() -> encoder.getVelocity() < 10)
        );
    }

    public ICommand next() {
        return setRelativeState(getState().next());
    }

    public ICommand previous() {
        return setRelativeState(getState().previous());
    }

    public void fullRotation() {
        switch (getState()) {
            case BALL1 -> setPosition(BALL1_END);
            case BALL2 -> setPosition(BALL2_END);
            case BALL3 -> setPosition(BALL3_END);
        }
    }

    public RelativeState getState() {
        return state;
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

    public void update() {
        super.update();
        encoder.update();
    }
}
