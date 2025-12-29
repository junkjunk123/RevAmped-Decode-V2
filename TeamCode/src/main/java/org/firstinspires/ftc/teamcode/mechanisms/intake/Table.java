package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.SimpleStateMachine;
import org.firstinspires.ftc.teamcode.utils.commands.StateMachine;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class Table extends HwServo {
    public enum RelativeState {
        BALL0,
        BALL1,
        BALL2;

        public float[] getShootStates() {
            switch (this) {
                case BALL0 -> {
                    return new float[] {Table.BALL1, Table.BALL2, Table.BALL2_END};
                }
                case BALL1 -> {
                    return new float[] {Table.BALL2, Table.BALL0_REV2, Table.BALL1_END};
                }
                case BALL2 -> {
                    return new float[] {Table.BALL0_REV2, Table.BALL1_REV2, Table.BALL2_END};
                }
            }

            return null;
        }

        public float target() {
            switch (this) {
                case BALL0 -> {
                    return Table.BALL0;
                }
                case BALL1 -> {
                    return Table.BALL1;
                }
                default -> {
                    return Table.BALL2;
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

    public static float BALL0;
    public static float BALL1;
    public static float BALL2;
    public static float BALL0_END;
    public static float BALL1_END;
    public static float BALL2_END;
    public static float BALL0_REV2;
    public static float BALL1_REV2;
    public static float FULL_REVOLUTION;
    public static double MS_PER_REVOLUTION = 1000;
    private final StateMachine<RelativeState> stateHandler = new SimpleStateMachine<>(RelativeState.BALL1);
    private final Encoder encoder;
    private final AtomicReference<Double> distance = new AtomicReference<>(0.0);

    public Table(HardwareMap hwMap, Encoder rawEncoder) {
        super(hwMap, "table");
        this.encoder = rawEncoder;
    }

    public ICommand one() {
        return setRelativeState(RelativeState.BALL0);
    }

    public ICommand two() {
        return setRelativeState(RelativeState.BALL1);
    }

    public ICommand three() {
        return setRelativeState(RelativeState.BALL2);
    }

    public ICommand reset() {
        return two();
    }

    public ICommand setState(int state) {
        return setRelativeState(RelativeState.values()[state]);
    }

    public ICommand setState(Supplier<Integer> state) {
        return setRelativeState(() -> RelativeState.values()[state.get()]);
    }

    public ICommand setRelativeState(Supplier<RelativeState> relativeState) {
        RelativeState[] state = new RelativeState[1];
        return new Conditional(
                () -> getState() == relativeState.get(),
                new Instant(() -> setPosition(getState().target())),
                stateHandler.runTransition(
                        new Sequential(
                                new Instant(() -> {
                                    state[0] = relativeState.get();
                                    distance.set(Math.abs(state[0].target() - getPosition()));
                                    setPosition(state[0].target());
                                }),
                                new Race(
                                        new Sequential(
                                                new Wait(250),
                                                new WaitUntil(() -> Math.abs(encoder.getVelocity()) < 10)
                                        ),
                                        new Wait(Math.abs(distance.get() / FULL_REVOLUTION * MS_PER_REVOLUTION))
                                )
                        ),
                        () -> state[0]
                )
        );
    }

    public ICommand setRelativeState(RelativeState relativeState) {
        return setRelativeState(() -> relativeState);
    }

    public ICommand next() {
        return setRelativeState(getState().next());
    }

    public ICommand previous() {
        return setRelativeState(getState().previous());
    }

    public ICommand fullRotation() {
        return switch (getState()) {
            case BALL0 -> setPos(BALL0_END);
            case BALL1 -> setPos(BALL1_END);
            case BALL2 -> setPos(BALL2_END);
        };
    }

    public ICommand setPos(float pos) {
        return new Conditional(
                () -> atPos(pos),
                new Command(),
                new Sequential(
                        new Instant(() -> {
                            setPosition(pos);
                            distance.set(Math.abs(pos - getPosition()));
                        }),
                        new Race(
                                new Sequential(
                                        new Wait(250),
                                        new WaitUntil(() -> Math.abs(encoder.getVelocity()) < 10)
                                ),
                                new Wait(Math.abs(distance.get() / FULL_REVOLUTION * MS_PER_REVOLUTION))
                        )
                )
        );
    }

    public RelativeState getState() {
        return stateHandler.getCurrentState();
    }

    public static void setValues(float BALL_0, float BALL_1, float BALL_0_END) {
        BALL0 = BALL_0;
        BALL1 = BALL_1;
        float diff = BALL_1 - BALL_0;
        BALL2 = diff + BALL1;
        FULL_REVOLUTION = diff * 3;
        BALL0_REV2 = BALL_0 + FULL_REVOLUTION;
        BALL1_REV2 = BALL_1 + FULL_REVOLUTION;
        BALL0_END = BALL_0_END;
        BALL1_END = BALL_0_END + diff;
        BALL2_END = BALL1_END + diff;
    }

    public void update() {
        super.update();
    }

    public StateMachine<RelativeState> getStateHandler() {
        return stateHandler;
    }

    public Encoder getEncoder() {
        return encoder;
    }
}
