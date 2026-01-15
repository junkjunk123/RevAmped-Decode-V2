package org.firstinspires.ftc.teamcode.mechanisms.lift;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utils.hardware.HwCRServo;

public class Lift extends HwCRServo {
    public enum LiftState {
        REST,
        RETRACT,
        LIFT
    }

    public static int LIFT;
    public static double P;
    public static double I;
    public static double D;
    public static double F;

    private final PIDFController controller;

    public Lift(HardwareMap hardwareMap, Encoder encoder) {
        super(hardwareMap, encoder, "lift_left", "lift_right");
        controller = new PIDFController(new PIDFCoefficients(P, I, D, F));
    }

    private LiftState state = LiftState.REST;

    public void rest() {
        if (state == LiftState.REST) return;
        controller.reset();
        state = LiftState.REST;
        setPower(0);
    }

    public void lift() {
        if (state == LiftState.LIFT) return;
        controller.reset();
        state = LiftState.LIFT;
    }

    public void retract() {
        if (state == LiftState.RETRACT) return;
        controller.reset();
        state = LiftState.RETRACT;
    }

    @Override
    public void update() {
        super.update();

        switch (state) {
            case LIFT -> {
                controller.updateFeedForwardInput(1);
                controller.updateError(LIFT - getPosition());
                setPower(controller.run());
            }
            case RETRACT -> {
                controller.updateFeedForwardInput(0);
                controller.updateError(-getPosition());
                setPower(controller.run() * 0.8);

                if (getPosition() <= 5) rest();
            }
        }
    }

    public PIDFController getController() {
        return controller;
    }
}