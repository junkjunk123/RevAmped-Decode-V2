package org.firstinspires.ftc.teamcode.mechanisms.lift;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.commands.WaitUntil;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.hardware.EncoderImpl;
import org.firstinspires.ftc.teamcode.utils.hardware.HwCRServo;

public class Lift extends HwCRServo {
    public enum LiftState {
        REST,
        LIFT
    }

    public Lift(HardwareMap hardwareMap, Encoder encoder) {
        super(hardwareMap, new EncoderImpl(encoder), "lift_left", "lift_right");
        hardware[0].setDirection(DcMotorSimple.Direction.FORWARD);
        hardware[1].setDirection(DcMotorSimple.Direction.REVERSE);
        setEncoderBase(getEncoder().getPosition());
    }

    private LiftState state = LiftState.REST;
    public static int FULLY_RAISED;

    public void rest() {
        if (state == LiftState.REST) return;
        state = LiftState.REST;
        setPower(0);
    }

    public ICommand lift() {
        return new Lazy(() -> {
            if (state == LiftState.LIFT) return Commands.NOOP;
            return new Sequential(
                    new Instant(() -> {
                        state = LiftState.LIFT;
                        setPower(1.0f);
                    }),
                    new Wait(500),
                    new WaitUntil(() -> Math.abs(Math.abs(getPosition()) - FULLY_RAISED) < 8)
                            .timeoutAfter(9000),
                    new Instant(this::rest)
            );
        });
    }
}