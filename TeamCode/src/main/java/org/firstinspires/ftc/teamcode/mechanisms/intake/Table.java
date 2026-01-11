package org.firstinspires.ftc.teamcode.mechanisms.intake;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.AtomicReadOnce;
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
    private final HwServo tableServo2;

    public Table(HardwareMap hwMap, Encoder rawEncoder) {
        super(hwMap, "table");
        this.encoder = rawEncoder;
        tableServo2 = new HwServo(hwMap, "table2");
    }

    public Command zero() {
        return setRelativeState(RelativeState.BALL0);
    }

    public Command one() {
        return setRelativeState(RelativeState.BALL1);
    }

    public Command two() {
        return setRelativeState(RelativeState.BALL2);
    }

    public Command reset() {
        return one();
    }

    public Command setState(int state) {
        return setRelativeState(RelativeState.values()[state]);
    }

    public Command setState(Supplier<Integer> state) {
        return setRelativeState(() -> RelativeState.values()[state.get()]);
    }

    public Command setRelativeState(Supplier<RelativeState> relativeState) {
        AtomicReadOnce<RelativeState> stateVal = new AtomicReadOnce<>(relativeState);
        return lazy(() -> {
           if (atPos(stateVal.read().target())) return Command.NOOP;
           return stateHandler.runTransition(
                   sequential(
                           instant(() -> {
                               distance.set(Math.abs(stateVal.read().target() - getPosition()));
                               setPosition(stateVal.read().target());
                           }),
                           race(
                                   sequential(
                                           waitMs(250.0),
                                           waitUntil(() -> Math.abs(encoder.getVelocity()) < 10)
                                   ),
                                   waitMs(Math.abs(distance.get() / FULL_REVOLUTION * MS_PER_REVOLUTION))
                           )
                   ),
                   stateVal::read
           );
        });
    }

    public Command setRelativeState(RelativeState relativeState) {
        return setRelativeState(() -> relativeState);
    }

    public Command next() {
        AtomicReadOnce<RelativeState> reader = pendingStateReader();
        return setRelativeState(() -> reader.read().next());
    }

    public Command previous() {
        AtomicReadOnce<RelativeState> reader = pendingStateReader();
        return setRelativeState(() -> reader.read().previous());
    }

    public Command fullRotation() {
        AtomicReadOnce<RelativeState> reader = pendingStateReader();
        return setPos(() -> switch (reader.read()) {
            case BALL0 -> BALL0_END;
            case BALL1 -> BALL1_END;
            case BALL2 -> BALL2_END;
        });
    }

    public Command setPos(float pos) {
        return setPos(() -> pos);
    }

    public Command setPos(Supplier<Float> pos) {
        float[] position = new float[1];
        return conditional(
                () -> atPos(pos.get()),
                Command.NOOP,
                sequential(
                        instant(() -> {
                            position[0] = pos.get();
                            distance.set(Math.abs(position[0] - getPosition()));
                            setPosition(position[0]);
                        }),
                        race(
                                sequential(
                                        waitMs(250.0),
                                        waitUntil(() -> Math.abs(encoder.getVelocity()) < 10)
                                ),
                                waitMs(Math.abs(distance.get() / FULL_REVOLUTION * MS_PER_REVOLUTION))
                        )
                )
        );
    }

    public RelativeState getState() {
        return stateHandler.getCurrentState();
    }

    public RelativeState pendingState() {
        return stateHandler.getPendingState();
    }

    public AtomicReadOnce<RelativeState> pendingStateReader() {
        return new AtomicReadOnce<>(this::pendingState);
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
    public static void setValues(float BALL_0, float BALL_1, float BALL_2, float BALL_0_END, float BALL_1_END, float BALL_2_END, float FULL_REVOLUTION_TICKS){
        FULL_REVOLUTION = FULL_REVOLUTION_TICKS;
        BALL0 = BALL_0;
        BALL1 = BALL_1;
        BALL2 = BALL_2;
        BALL0_END = BALL_0_END;
        BALL1_END = BALL_1_END;
        BALL2_END = BALL_2_END;
        BALL0_REV2 = BALL_0 + FULL_REVOLUTION;
        BALL1_REV2 = BALL_1 + FULL_REVOLUTION;
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

    @Override
    public boolean setPosition(double pos) {
        if (tableServo2 != null) tableServo2.setPosition(pos);
        return super.setPosition(pos);
    }
}
