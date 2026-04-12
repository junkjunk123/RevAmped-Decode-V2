package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Infinite;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Race;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.math.calc.Integrator;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.Loop;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Channels;
import org.firstinspires.ftc.teamcode.utils.commands.channel.Speaker;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.hardware.HwDigitalDevice;
import org.firstinspires.ftc.teamcode.utils.hardware.HwMotor;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.DoubleSupplier;

@Config
public class Turret extends HwMotor {
    public static double TURRET_OFFSET = 1.0; //turret center offset to center of robot
    public static double RAD_LIMIT;
    public static double TICKS_LIMIT;
    public static double FULL_ROTATION;

    public static void updateFullRotation() {
        FULL_ROTATION = TICKS_LIMIT / RAD_LIMIT * 2 * Math.PI;
    }

    public sealed interface MoveState permits MoveState.AutoTrack, MoveState.CloseAuto, MoveState.Deenergize, MoveState.FarPreset, MoveState.MoveTo, MoveState.PresetState, MoveState.SwitchReset, MoveState.Track {
        Deenergize DEENERGIZE = new Deenergize();
        CloseAuto CLOSE_AUTO = new CloseAuto();
        SwitchReset SWITCH_RESET = new SwitchReset();
        AutoTrack AUTO_TRACK = new AutoTrack();

        enum PresetState implements MoveState {
            LEFT_135,
            LEFT_90,
            LEFT_45,
            REST,
            RIGHT_45,
            RIGHT_90,
            RIGHT_135;

            public int target() {
                return (ordinal() - REST.ordinal()) * Turret.ticksPerRotation() / 8;
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
        record
        Track(int target) implements MoveState {}
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

        final class SwitchReset implements MoveState {
            @Override
            public int target() {
                return 10;
            }
        }

        final class AutoTrack implements MoveState {
            @Override
            public int target() {
                return 10;
            }
        }

        final class FarPreset implements MoveState {
            private final int target;

            public FarPreset(AllianceColor allianceColor) {
                target = allianceColor == AllianceColor.Red ? FAR_PRESET_RED : FAR_PRESET_BLUE;
            }

            @Override
            public int target() {
                return target;
            }
        }
    }

    public static int AUTO_PRELOADS;
    public static int AUTO_SET_1;
    public static int AUTO_SET_2;
    public static int AUTO_SET_3;
    public static int FAR_AUTO;
    public static int FIFTEEN_BALL_PRELOADS;
    public static int FIFTEEN_OBELISK_DETECTION;
    public static int UNSORTED_AUTO_PRELOADS;
    public static int UNSORTED_GATE;
    public static int UNSORTED_SET_1;
    public static int UNSORTED_SET_2;
    public static int UNSORTED_SET_3;
    public static int UNSORTED_SET_4;
    public static int UNSORTED_SET_5;
    public static int UNSORTED_FINAL;
    public static double P;
    public static double I;
    public static double D;
    public static double F;
    public static double P_SECONDARY;
    public static double I_SECONDARY;
    public static double D_SECONDARY;
    public static double F_SECONDARY;
    public static double P_RESET;
    public static double I_RESET;
    public static double D_RESET;
    public static double F_RESET;
    public static double TRACK_D;
    public static int PIDF_SWITCH;
    public static double MS_PER_REVOLUTION = 2000;
    public static int FAR_PRESET_RED = -173;
    public static int FAR_PRESET_BLUE = 150;
    public static int LEGAL_FAR_BLUE = 400;
    public static int ILLEGAL_FAR_BLUE = 680;
    public static int ILLEGAL_FAR_RED = -720;
    public static int LEGAL_FAR_RED = 380;

    public static int ticksPerRotation() {
        return (int) (TICKS_LIMIT * (Math.PI * 2 / RAD_LIMIT));
    }

    public final HwDigitalDevice limitSwitch;
    private final PIDFController controller;
    private final PIDFController secondaryController;
    private final PIDFController resetController;
    private final PIDFController trackController;
    private MoveState moveState = MoveState.PresetState.REST;
    private final AtomicInteger distance = new AtomicInteger(0);
    private boolean useSecondary = false;
    private DoubleSupplier limelightError;

    public Turret(HardwareMap hardwareMap, Encoder encoder) {
        super(hardwareMap, false, "turret");
        setEncoder(encoder);
        controller = new PIDFController(new PIDFCoefficients(P, I, D, F));
        secondaryController = new PIDFController(new PIDFCoefficients(P_SECONDARY, I_SECONDARY, D_SECONDARY, F_SECONDARY));
        resetController = new PIDFController(new PIDFCoefficients(P_RESET, I_RESET, D_RESET, F_RESET));
        trackController = new PIDFController(new PIDFCoefficients(P, I, TRACK_D, F));
        limitSwitch = new HwDigitalDevice(hardwareMap, "turret_switch").flip();
        updateTargetPosition(0);
        resetController.setTargetPosition(0);
        setDirection(DcMotorSimple.Direction.FORWARD);
        if (Globals.isTeleOp && Globals.turretStartPos > 1)
            setEncoderBase((int) (getEncoder().getPosition() - Globals.turretStartPos));
        else
            setEncoderBase(getEncoder().getPosition());
        Globals.turretStartPos = 0;
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
        trackController.setTargetPosition(target);
        useSecondary = false;
    }

    public void move(MoveState moveState) {
        if (!(moveState instanceof MoveState.SwitchReset)) updateTargetPosition(moveState.target());
        else updateTargetPosition(moveState.target() * (int) (-1 * Math.signum(getPosition())));
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
                new Sequential(
                        runToState(MoveState.SWITCH_RESET),
                        new Wait(500)
                ),
                new Sequential(
                        new WaitUntil(limitSwitch::state),
                        new Instant(() -> {
                            if (moveState instanceof MoveState.SwitchReset) resetPosition(moveState.target());
                            else resetPosition();
                            setTargetPosition(0);
                        })
                )
        );
    }

    public ICommand reached() {
        return new Race(
                new WaitUntil(() -> Math.abs(getVelocity()) < 10 && Math.abs(getTargetPosition() - getPosition()) < 25),
                new Wait(Math.abs(distance.get() / FULL_ROTATION * MS_PER_REVOLUTION))
        );
    }

    public double predictedMoveTime() {
        return distance.get() / FULL_ROTATION * MS_PER_REVOLUTION;
    }

    @Override
    public void update() {
        super.update();
        limitSwitch.update();
        trackController.setCoefficients(new PIDFCoefficients(P, I, TRACK_D, F));

        if (deenergized()) return;

        double error = getTargetPosition() - getPosition();

        if (moveState instanceof MoveState.SwitchReset) {
            setPower(updateController(resetController, error));
            return;
        }

        if (moveState instanceof MoveState.Track) {
            updateController(trackController, error);
            setPower(trackController.run());
            return;
        }

        if (moveState instanceof MoveState.AutoTrack) {
            if (limelightError == null) throw new IllegalArgumentException("DIE");
            updateController(trackController, getLimelightError());
            setPower(trackController.run());
            return;
        }

        if (!useSecondary && Math.abs(error) > PIDF_SWITCH) {
            setPower(updateController(secondaryController, error));
            return;
        }

        useSecondary = true;
        setPower(updateController(controller, error));
    }

    private double updateController(PIDFController controller, double error) {
        controller.updatePosition(getPosition());
        controller.updateFeedForwardInput(Math.signum(error));
        return controller.run();
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

    public void unsortedAutoSet(int setNum) {
        switch (setNum + 1) {
            case 1 -> setTargetPosition(UNSORTED_SET_1);
            case 2 -> setTargetPosition(UNSORTED_SET_2);
            case 3 -> setTargetPosition(UNSORTED_SET_3);
            case 4 -> setTargetPosition(UNSORTED_SET_4);
            case 5 -> setTargetPosition(UNSORTED_SET_5);
            default -> setTargetPosition(UNSORTED_AUTO_PRELOADS);
        }
    }

    @Override
    public Speaker<String> test() {
        setPower(0.3);
        Integrator current = new Integrator();
        Integrator encoder = new Integrator();
        double nominalCurrent = get().getCurrent(CurrentUnit.AMPS);
        return new Speaker<>(c ->
                new Sequential(
                        new Race(
                                new Wait(2000),
                                new Infinite(() -> {
                                    current.update(Math.abs(nominalCurrent - get().getCurrent(CurrentUnit.AMPS)));
                                    encoder.update(Math.abs(getEncoder().getVelocity()));
                                }),
                                new Loop(
                                        new Sequential(
                                                new Wait(500),
                                                new Instant(() -> setPower(-0.3))
                                        ),
                                        4
                                )
                        ),
                        new Instant(() -> setPower(0)),
                        Channels.send(c, () -> {
                            if (current.getIntegral() > 0.08)
                                return this + "MOTOR TEST PASS: Current draw normal.";
                            else
                                return this + "MOTOR TEST FAIL: Current draw too low!";
                        }),
                        Channels.send(c, () -> {
                            if (encoder.getIntegral() > 15)
                                return this + "MOTOR TEST PASS: Encoder counts normal.";
                            else
                                return this + "MOTOR TEST FAIL: Encoder counts too low!";
                        })
                )
        );
    }

    public void setLimelightError(DoubleSupplier limelightError) {
        this.limelightError = limelightError;
    }

    private double getLimelightError() {
        double angularError = limelightError.getAsDouble();
        return FULL_ROTATION / Math.PI / 2 * angularError;
    }

    public void farPreset() {
        move(new MoveState.FarPreset(Globals.allianceColor));
    }
}