package org.firstinspires.ftc.teamcode.revamped.mechanisms.intake;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.matchType;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.scope;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitSeconds;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.revamped.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.EncoderImpl;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.HwServo;

import dev.frozenmilk.dairy.mercurial.continuations.Actors;
import dev.frozenmilk.dairy.mercurial.continuations.Closure;
import dev.frozenmilk.dairy.mercurial.continuations.Continuations;
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels;
import dev.frozenmilk.dairy.mercurial.continuations.channels.Sender;
import dev.frozenmilk.dairy.mercurial.continuations.registers.VarRegister;

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
    }

    public sealed interface MoveState permits MoveState.Hold, MoveTo {
        double target();

        record Hold(double target) implements MoveState {}
    }

    public record MoveTo(double target, Sender<Boolean> reachedTx) implements MoveState {}

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

    private RelativeState initialShootState = RelativeState.BALL2;
    private final EncoderImpl encoder;
    private final Actors.Actor<MoveState, MoveState> movementActor;

    /**
     * @param hwMap hardwareMap
     * @param id    the ID of the servo as configured
     */
    public Table(HardwareMap hwMap, String id, Encoder rawEncoder) {
        super(hwMap, id);
        this.encoder = new EncoderImpl(rawEncoder);
        movementActor = Actors.actor(
                        () -> new MoveState.Hold(BALL1),

                        (state, message) -> message,

                        state ->
                                matchType(state)
                                        .branch(
                                                MoveTo.class,
                                                (move) ->
                                                        scope(dist -> {
                                                            VarRegister<Double> distRegister =
                                                                    dist.variable(() -> Math.abs(getPosition() - move.get().target()));
                                                                    return sequence(
                                                                            exec(
                                                                                    () -> setPosition(move.get().target())
                                                                            ),
                                                                            waitSeconds(0.25),
                                                                            Continuations.race(
                                                                                    waitUntil(() -> encoder.getVelocity() < 10),
                                                                                    waitSeconds(distRegister.get() / FULL_REVOLUTION * SECONDS_PER_ROTATION)
                                                                            ),
                                                                            exec(() -> Channels.send(() -> Boolean.TRUE, () -> move.get().reachedTx()))
                                                                    );
                                                                }
                                                        )

                                        )
                                        .branch(
                                                MoveState.Hold.class,
                                                (hold) -> exec(() -> setPosition(hold.get().target()))
                                        )
                                        .assertExhaustive()
                );
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

    public Closure one(Sender<Boolean> reached) {
        initialShootState = RelativeState.BALL1;
        return sendToPosition(BALL1, reached);
    }

    public Closure two(Sender<Boolean> reached) {
        initialShootState = RelativeState.BALL2;
        return sendToPosition(BALL2, reached);
    }

    public Closure three(Sender<Boolean> reached) {
        initialShootState = RelativeState.BALL3;
        return sendToPosition(BALL3, reached);
    }

    public void reset() {
        two();
    }

    public Closure reset(Sender<Boolean> reached) {
        return two(reached);
    }

    public RelativeState getState() {
        return initialShootState;
    }

    public Closure sendToPosition(float position, Sender<Boolean> reached) {
        return Channels.send(() -> new MoveTo(position, reached), movementActor::tx);
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
        encoder.update();
    }
}
