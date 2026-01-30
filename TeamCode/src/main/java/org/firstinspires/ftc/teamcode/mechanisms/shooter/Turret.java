package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.hardware.HwDigitalDevice;
import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;

import java.util.concurrent.atomic.AtomicInteger;

@Config
public class Turret extends HwMotor {
    public static double RAD_LIMIT;
    public static double TICKS_LIMIT;
    public static double FULL_ROTATION;

    public static void updateFullRotation() {
        FULL_ROTATION = TICKS_LIMIT / RAD_LIMIT * 2 * Math.PI;
    }

    public sealed interface MoveState permits MoveState.CloseAuto, MoveState.Deenergize, MoveState.MoveTo, MoveState.PresetState {
        Deenergize DEENERGIZE = new Deenergize();
        CloseAuto CLOSE_AUTO = new CloseAuto();
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
        final class Deenergize implements MoveState {
            @Override
            public int target() {
                return 0;
            }
        }
        final class CloseAuto implements MoveState {
            @Override
            public int target() {
                return AUTO_PRELOADS;
            }
        }
    }

    public static int AUTO_PRELOADS;
    public static int AUTO_SET_1;
    public static int AUTO_SET_2;
    public static int AUTO_SET_3;
    public static int FAR_AUTO;
    public static int UNSORTED_AUTO_PRELOADS;
    public static int UNSORTED_GATE;
    public static int UNSORTED_FINAL;
    public static double P;
    public static double I;
    public static double D;
    public static double F;
    public static double P_SECONDARY;
    public static double I_SECONDARY;
    public static double D_SECONDARY;
    public static double F_SECONDARY;
    public static int PIDF_SWITCH;
    public static double MS_PER_REVOLUTION = 2000;
    public static int ticksPerRotation() {
        return (int) (TICKS_LIMIT * (Math.PI * 2 / RAD_LIMIT));
    }
    public final HwDigitalDevice limitSwitch;
    private final PIDFController controller;
    private final PIDFController secondaryController;
    private MoveState moveState = MoveState.PresetState.REST;
    private final AtomicInteger distance = new AtomicInteger(0);
    private boolean useSecondary = false;

    public Turret(HardwareMap hardwareMap) {
        super(hardwareMap, "turret");
        controller = new PIDFController(new PIDFCoefficients(P, I, D, F));
        secondaryController = new PIDFController(new PIDFCoefficients(P_SECONDARY, I_SECONDARY, D_SECONDARY, F_SECONDARY));
        limitSwitch = new HwDigitalDevice(hardwareMap, "turret_switch").flip();
        updateTargetPosition(0);
        setDirection(DcMotorSimple.Direction.REVERSE);
        setEncoderBase(getEncoder().getPosition());
        invalidateCache();
    }

    public void setTargetPosition(int position) {
        move(new MoveState.MoveTo(position));
    }

    public void angleTo(double radians) {
        setTargetPosition((int) (radians / (Math.PI * 2) * FULL_ROTATION));
    }

    public void finetune(int ticks) {
        setTargetPosition(getTargetPosition() + ticks);
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
        secondaryController.setTargetPosition(target);
        useSecondary = false;
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
        return new Race(
                runToState(MoveState.PresetState.REST),
                new Sequential(
                        new WaitUntil(limitSwitch::state),
                        new Instant(this::resetPosition)
                )
        );
    }

    public ICommand reached() {
        return new Race(
                new WaitUntil(() -> Math.abs(getVelocity()) < 10 && Math.abs(getTargetPosition() - getPosition()) < 25),
                new Wait(Math.abs(distance.get() / FULL_ROTATION * MS_PER_REVOLUTION))
        );
    }

    @Override
    public void update() {
        super.update();
        limitSwitch.update();

        if (deenergized()) return;

        double error = getTargetPosition() - getPosition();

        if (!useSecondary && Math.abs(error) > PIDF_SWITCH) {
            controller.updatePosition(getPosition());
            controller.updateFeedForwardInput(Math.signum(error));
            setPower(controller.run());
            return;
        }

        useSecondary = true;
        secondaryController.updatePosition(getPosition());
        secondaryController.updateFeedForwardInput(Math.signum(error));
        setPower(secondaryController.run());
    }

    public ICommand prepareForLift() {
        return new Sequential(
                runToPos(FAR_AUTO),
                new Instant(() -> {
                    deenergize();
                    moveState = MoveState.DEENERGIZE;
                })
        );
    }

    public boolean deenergized() {
        return moveState.equals(MoveState.DEENERGIZE);
    }

    public PIDFController getController() {
        return controller;
    }

    public PIDFController getSecondaryController() {
        return secondaryController;
    }
}