package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class Splitter extends HwServo {
    public static float ACTIVATED;
    public static float NEUTRAL;
    public static double NEUTRAL_DELAY;
    public static double ACTIVATE_DELAY;

    private enum State {
        ACTIVATED,
        NEUTRAL
    }
    private State state = State.ACTIVATED;

    public Splitter(HardwareMap hwMap) {
        super(hwMap, "splitter");
    }

    public ICommand neutral() {
        return new Lazy(() -> {
            if (state == State.NEUTRAL) return Commands.NOOP;
            return new Sequential(
                    new Instant(this::setPositionNeutral),
                    new Wait(NEUTRAL_DELAY)
            );
        });
    }

    public void setPositionNeutral() {
        setPosition(NEUTRAL);
        state = State.NEUTRAL;
    }

    public void setPositionActivated() {
        setPosition(ACTIVATED);
        state = State.ACTIVATED;
    }

    public ICommand activate() {
        return new Lazy(() -> {
            if (state == State.ACTIVATED) return Commands.NOOP;
            return new Sequential(
                    new Instant(this::setPositionActivated),
                    new Wait(ACTIVATE_DELAY)
            );
        });
    }

    public String getState() {
        return state.name();
    }
}
