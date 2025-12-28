package org.firstinspires.ftc.teamcode.mechanisms.shooter;
import android.media.audiofx.PresetReverb;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwDigitalDevice;
import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;

public class Turret extends HwMotor {
    public static double RAD_LIMIT;
    public static double TICKS_LIMIT;
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

    public void runToPos(int position) {
        controller.setTargetPosition(position);
    }

    public int getTargetPosition() {
        return (int) controller.getTargetPosition();
    }

    public void move(MoveState moveState) {
        runToPos(moveState.target());
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

    public boolean reached() {
        return Math.abs(getVelocity()) < 10 && Math.abs(getTargetPosition() - getPosition()) < 25;
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
