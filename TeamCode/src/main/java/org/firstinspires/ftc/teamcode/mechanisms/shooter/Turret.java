package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwDigitalDevice;
import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;

import java.util.concurrent.atomic.AtomicInteger;

public class Turret extends HwMotor {
    public static double RAD_LIMIT;
    public static double TICKS_LIMIT;
    public static double FULL_ROTATION;

    public static void updateFullRotation() {
        FULL_ROTATION = TICKS_LIMIT / RAD_LIMIT * 2 * Math.PI;
    }

    public static int AUTO_PRELOADS;
    public static int AUTO_SET_1;
    public static int AUTO_SET_2;
    public static int AUTO_SET_3;
    public static int FAR_AUTO;
    public static double P;
    public static double I;
    public static double D;
    public static double F;
    public static int startPos;
    public static double MS_PER_REVOLUTION = 2000;
    private final AtomicInteger distance = new AtomicInteger(0);

    public sealed interface MoveState permits MoveState.MoveTo, MoveState.PresetState {
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

        int target();

        record MoveTo(int target) implements MoveState {}
    }

    public static int ticksPerRotation() {
        return (int) (TICKS_LIMIT * (Math.PI * 2 / RAD_LIMIT));
    }

    public final HwDigitalDevice limitSwitch;
    private final PIDFController controller;
    private MoveState moveState = MoveState.PresetState.REST;

    public Turret(HardwareMap hardwareMap) {
        super(hardwareMap, "turret");
        controller = new PIDFController(new PIDFCoefficients(P, I, D, F));
        limitSwitch = new HwDigitalDevice(hardwareMap, "turret_switch");
        resetPosition(startPos);
        setDirection(DcMotorSimple.Direction.REVERSE);
        limitSwitch.setFlipped(true);
    }

    public void setTargetPosition(int position) {
        move(new MoveState.MoveTo(position));
    }

    public ICommand runToPos(int position) {
        return new Sequential(
                new Instant(() -> setTargetPosition(position)),
                reached()
        );
    }

    public ICommand runToState(MoveState state) {
        return new Sequential(
                new Instant(() -> move(state)),
                reached()
        );
    }

    public int getTargetPosition() {
        return (int) controller.getTargetPosition();
    }

    private void updateTargetPosition(int target) {
        distance.set(Math.abs(target - getTargetPosition()));
        controller.setTargetPosition(target);
    }

    public void move(MoveState moveState) {
        updateTargetPosition(moveState.target());
        this.moveState = moveState;
    }

    public MoveState getMoveState() {
        return moveState;
    }

    public void next() {
        if (moveState instanceof MoveState.PresetState p)
            move(p.next());
        else
            move(MoveState.PresetState.REST);
    }

    public void previous() {
        if (moveState instanceof MoveState.PresetState p)
            move(p.previous());
        else
            move(MoveState.PresetState.REST);
    }

    public ICommand resetTurret() {
        return new Sequential(
                new Instant(() -> move(MoveState.PresetState.REST)),
                new WaitUntil(limitSwitch::state),
                new Instant(this::resetPosition)
        );
    }

    public ICommand reached() {
        return new Race(
                new WaitUntil(() -> Math.abs(getVelocity()) < 10 && Math.abs(getTargetPosition() - getPosition()) < 25),
                new Wait(distance.get() / FULL_ROTATION * MS_PER_REVOLUTION)
        );
    }

    @Override
    public void update() {
        super.update();
        limitSwitch.update();
        controller.updatePosition(getPosition());
        controller.updateFeedForwardInput(Math.signum(getTargetPosition() - getPosition()));
        setPower(controller.run());
    }
}
