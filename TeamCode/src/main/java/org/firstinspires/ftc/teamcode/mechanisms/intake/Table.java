package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmodes.teleop.Tele;
import org.firstinspires.ftc.teamcode.utils.AtomicReadOnce;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.commands.SimpleStateMachine;
import org.firstinspires.ftc.teamcode.utils.commands.StateMachine;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.hardware.EncoderImpl;
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
    private final EncoderImpl encoder;
    private final AtomicReference<Double> distance = new AtomicReference<>(0.0);
    private final HwServo tableServo2;
    private boolean atRelativeState = true;

    public Table(HardwareMap hwMap, Encoder rawEncoder) {
        super(hwMap, "table");
        this.encoder = new EncoderImpl(rawEncoder);
        tableServo2 = new HwServo(hwMap, "table2");
    }

    public ICommand zero() {
        return setRelativeState(RelativeState.BALL0);
    }

    public ICommand one() {
        return setRelativeState(RelativeState.BALL1);
    }

    public ICommand two() {
        return setRelativeState(RelativeState.BALL2);
    }

    public ICommand reset() {
        return one();
    }

    public ICommand setState(int state) {
        return setRelativeState(RelativeState.values()[state]);
    }

    public ICommand setState(Supplier<Integer> state) {
        return setRelativeState(() -> RelativeState.values()[state.get()]);
    }

    public ICommand setRelativeState(Supplier<RelativeState> relativeState) {
        AtomicReadOnce<RelativeState> stateVal = new AtomicReadOnce<>(relativeState);
        AtomicReadOnce<Double> accelTime = getAccelerationTime(true);
        Tele.state = stateVal;
        return new Lazy(() -> {
           if (atPos(stateVal.read().target())) return Commands.NOOP;
           return stateHandler.runTransition(
                   new Sequential(
                           new Instant(() -> {
                               distance.set(Math.abs(stateVal.read().target() - getPosition()));
                               setPosition(stateVal.read().target());
                           }),
                           new Race(
                                   new Sequential(
                                           new Wait(accelTime.read()),
                                           new WaitUntil(() -> Math.abs(encoder.getVelocity()) < 10)
                                   ),
                                   new Wait(Math.abs(distance.get() / FULL_REVOLUTION * MS_PER_REVOLUTION))
                           ),
                           new Instant(() -> atRelativeState = true)
                   ),
                   stateVal::read
           );
        });
    }

    public ICommand setRelativeState(RelativeState relativeState) {
        return setRelativeState(() -> relativeState);
    }

    public ICommand next() {
        AtomicReadOnce<RelativeState> reader = pendingStateReader();
        return setRelativeState(() -> reader.read().next());
    }

    public ICommand previous() {
        AtomicReadOnce<RelativeState> reader = pendingStateReader();
        return setRelativeState(() -> reader.read().previous());
    }

    public ICommand fullRotation() {
        AtomicReadOnce<RelativeState> reader = pendingStateReader();
        return setPos(() -> switch (reader.read()) {
            case BALL0 -> BALL0_END;
            case BALL1 -> BALL1_END;
            case BALL2 -> BALL2_END;
        });
    }

    public ICommand setPos(float pos) {
        return setPos(() -> pos);
    }

    public ICommand setPos(Supplier<Float> pos) {
        float[] position = new float[1];
        AtomicReadOnce<Double> accelTime = getAccelerationTime(false);
        return new Conditional(
                () -> atPos(pos.get()),
                Commands.NOOP,
                new Sequential(
                        new Instant(() -> {
                            position[0] = pos.get();
                            distance.set(Math.abs(position[0] - getPosition()));
                            setPosition(position[0]);
                        }),
                        new Race(
                                new Sequential(
                                        new Wait(accelTime.read()),
                                        new WaitUntil(() -> Math.abs(encoder.getVelocity()) < 10)
                                ),
                                new Wait(Math.abs(distance.get() / FULL_REVOLUTION * MS_PER_REVOLUTION))
                        ),
                        new Instant(() -> atRelativeState = false)
                )
        );
    }

    private AtomicReadOnce<Double> getAccelerationTime(boolean targetRelative) {
        return new AtomicReadOnce<>(() -> {
            if (atRelativeState != targetRelative) return 600.0;
            else return 250.0;
        });
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

    public void update() {
        super.update();
        encoder.update();
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
        boolean moved = super.setPosition(pos);
        if (moved) encoder.reset();
        return moved;
    }
}
