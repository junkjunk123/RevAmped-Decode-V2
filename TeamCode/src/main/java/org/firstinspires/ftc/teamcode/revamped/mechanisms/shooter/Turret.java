package org.firstinspires.ftc.teamcode.revamped.mechanisms.shooter;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.matchType;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.noop;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.revamped.utils.hardware.HwDigitalDevice;
import org.firstinspires.ftc.teamcode.revamped.utils.hardware.HwMotor;

import dev.frozenmilk.dairy.mercurial.continuations.Actors;
import dev.frozenmilk.dairy.mercurial.continuations.channels.Channels;

public class Turret extends HwMotor {
    public static double RAD_LIMIT;
    public static double TICKS_LIMIT;
    public static int AUTO_PRELOADS;
    public static int AUTO_SET_1;
    public static int AUTO_SET_2;
    public static int AUTO_SET_3;
    public static int FAR_AUTO;
    public static float LL_DETECTION_RED;
    public static float LL_DETECTION_BLUE;
    public static double P;
    public static double I;
    public static double D;
    public static double F;

    public sealed interface MoveState permits MoveState.MoveTo, MoveState.Next, MoveState.PresetState, MoveState.Previous {
        enum PresetState implements MoveState {
            LEFT_135,
            LEFT_90,
            LEFT_45,
            REST,
            RIGHT_45,
            RIGHT_90,
            RIGHT_135;

            public int target() {
                return (ordinal() - REST.ordinal()) * Turret.ticksPerRotation() / values().length;
            }

            public PresetState next() {
                if (this == RIGHT_135) return RIGHT_135;
                return values()[ordinal() + 1];
            }

            public PresetState previous() {
                if (this == LEFT_135) return LEFT_135;
                return values()[ordinal() - 1];
            }
        }

        final class Next implements MoveState {
            public static Next INSTANCE = new Next();
        }

        final class Previous implements MoveState {
            public static Previous INSTANCE = new Previous();
        }

        record MoveTo(int target) implements MoveState {}
    }

    public static int ticksPerRotation() {
        return (int) (TICKS_LIMIT * (Math.PI * 2 / RAD_LIMIT));
    }

    public final HwDigitalDevice limitSwitch;
    private final PIDFController controller;
    private final Actors.Actor<MoveState, MoveState> moveStateActor;

    public Turret(HardwareMap hardwareMap) {
        super(hardwareMap, "turret");
        controller = new PIDFController(new PIDFCoefficients(P, I, D, F));
        limitSwitch = new HwDigitalDevice(hardwareMap, "turret_switch");
        moveStateActor = new Actors.Actor<>(
                () -> MoveState.PresetState.REST,
                (s, m) -> {
                    if (m instanceof MoveState.Next)
                        if (s instanceof MoveState.PresetState p)
                            return p.next();
                        else
                            return MoveState.PresetState.REST;
                    else if (m instanceof MoveState.Previous)
                        if (s instanceof MoveState.PresetState p)
                            return p.previous();
                        else
                            return MoveState.PresetState.REST;
                    return m;
                },
                state ->
                        matchType(state)
                                .branch(MoveState.PresetState.class, preset -> exec(() -> setTargetPosition(preset.get().target())))
                                .branch(MoveState.MoveTo.class, command -> exec(() -> setTargetPosition(command.get().target())))
                                .branch(MoveState.Next.class, t -> noop())
                                .branch(MoveState.Previous.class, t -> noop())
        );
    }

    public void setTargetPosition(int position) {
        controller.setTargetPosition(position + getEncoderBase());
    }

    public int getTargetPosition() {
        return (int) controller.getTargetPosition() - getEncoderBase();
    }

    public void preset(MoveState.PresetState presetState) {
        controller.reset();
        Channels.send(() -> presetState, moveStateActor::tx);
    }

    public void moveTo(int pos) {
        if (Math.abs(pos - getTargetPosition()) > 0.5)
            controller.reset();
        Channels.send(() -> new MoveState.MoveTo(pos), moveStateActor::tx);
    }

    public void nextPreset() {
        controller.reset();
        Channels.send(() -> MoveState.Next.INSTANCE, moveStateActor::tx);
    }

    public void previousPreset() {
        controller.reset();
        Channels.send(() -> MoveState.Previous.INSTANCE, moveStateActor::tx);
    }

    public void update() {
        super.update();
        limitSwitch.update();
        controller.updatePosition(getPosition());
        controller.updateFeedForwardInput(Math.signum(getTargetPosition() - getPosition()));
        setPower(controller.run());
    }
}
