package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.calc.Integrator;
import org.firstinspires.ftc.teamcode.utils.AtomicReadOnce;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.commands.SimpleStateMachine;
import org.firstinspires.ftc.teamcode.utils.commands.StateMachine;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channels;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Speaker;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.hardware.EncoderImpl;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;
import org.firstinspires.ftc.teamcode.utils.logging.DecodeLogger;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class Table extends HwServo {
    public enum RelativeState {
        BALL0,
        BALL1,
        BALL2;

        public float[] getShootStates() {
            float first = target() + SHOOT_INCREMENT;
            float secondIncrement = FULL_REVOLUTION / 3;
            float second = secondIncrement + first;
            float third = secondIncrement + second;
            return new float[] {first, second, third};
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

        public float getFullRotation() {
            switch (this) {
                case BALL0 -> {
                    return BALL0_END;
                }
                case BALL1 -> {
                    return BALL1_END;
                }
                default -> {
                    return BALL2_END;
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
    public static float SHOOT_INCREMENT;
    public static double MS_PER_REVOLUTION = 750;
    public static double SLOW_SHOOT_DELAY = 25;
    private final int VELOCITY_THRESHOLD = 20;
    private final int VELOCITY_THRESHOLD_2 = 15;
    private final StateMachine<RelativeState> stateHandler = new SimpleStateMachine<>(RelativeState.BALL1, "table");
    private final EncoderImpl encoder;
    private final AtomicReference<Double> distance = new AtomicReference<>(0.0);
    private final HwServo tableServo2;
    private boolean atRelativeState = true;
    private boolean useEncoder;

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
        return new Lazy(() -> {
           RelativeState targetState = stateVal.read();
           if (atPos(targetState.target())) return Commands.NOOP;
           return stateHandler.runTransition(
                   new Sequential(
                           new Instant(() -> setRelativeStateAndLog(targetState)),
                           useEncoder ? new Race(
                                   new Sequential(
                                           new Wait(accelTime.read()),
                                           new WaitUntil(() -> Math.abs(encoder.getVelocity()) < VELOCITY_THRESHOLD)
                                   ),
                                   new Wait(Math.min(Math.abs(distance.get() / FULL_REVOLUTION * MS_PER_REVOLUTION), MS_PER_REVOLUTION))
                           ) : new Wait(Math.min(Math.abs(distance.get() / FULL_REVOLUTION * MS_PER_REVOLUTION), MS_PER_REVOLUTION - 150)),
                           new Instant(() -> atRelativeState = true)
                   ),
                   () -> targetState
           );
        });
    }

    public void setStateCommandless(RelativeState relativeState) {
        RelativeState previous = stateHandler.getCurrentState();
        stateHandler.setCurrentState(relativeState);
        boolean moved = setPosition(relativeState.target());
        atRelativeState = true;
        if (moved || previous != relativeState) {
            logRelativeState(relativeState);
        }
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

    public ICommand shoot() {
        AtomicReadOnce<RelativeState> reader = pendingStateReader();
        return new Sequential(
                new Instant(() -> setPosition(reader.read().getFullRotation())),
                new Race(
                        new Wait(1000),
                        new Sequential(
                                new Wait(600),
                                new WaitUntil(() -> encoder.getVelocity() < VELOCITY_THRESHOLD_2)
                        )
                ),
                new Instant(() -> atRelativeState = false)
        );
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
                        useEncoder ? new Race(
                                new Sequential(
                                        new Wait(accelTime.read()),
                                        new WaitUntil(() -> Math.abs(encoder.getVelocity()) < VELOCITY_THRESHOLD)
                                ),
                                new Wait(Math.abs(distance.get() / FULL_REVOLUTION * MS_PER_REVOLUTION))
                        ) : new Wait(Math.abs(distance.get() / FULL_REVOLUTION * MS_PER_REVOLUTION)),
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

    public void setUseEncoder(boolean useEncoder) {
        this.useEncoder = useEncoder;
    }

    public Speaker<String> test() {
        setPosition(BALL1);
        Integrator encoder = new Integrator();
        return new Speaker<>(c ->
                new Sequential(
                        new Race(
                                new Wait(5000),
                                new Infinite(() -> encoder.update(Math.abs(this.encoder.rawVelocity())))
                        ),
                        Channels.send(c, () -> {
                            if (encoder.getIntegral() > 15)
                                return this + "MOTOR TEST PASS: Encoder counts normal.";
                            else
                                return this + "MOTOR TEST FAIL: Encoder counts too low!";
                        })
                )
        );
    }

    private void setRelativeStateAndLog(RelativeState state) {
        distance.set(Math.abs(state.target() - getPosition()));
        setPosition(state.target());
        logRelativeState(state);
    }

    private void logRelativeState(RelativeState state) {
        DecodeLogger.get().info("table", "TABLE_STATE_SET",
                "state", state.name(),
                "pos", state.target());
    }
}
