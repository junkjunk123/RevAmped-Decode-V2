package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Lazy;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class IntakeGate extends HwServo {
    public static float OPEN;
    public static float CLOSE;

    private enum GateState {
        OPEN,
        CLOSE
    }
    private GateState gateState = GateState.OPEN;

    public IntakeGate(HardwareMap hwMap) {
        super(hwMap, "gate");
    }

    public String getGateState() {
        return gateState.name();
    }

    public ICommand open() {
        return new Lazy(() -> {
            if (gateState == GateState.OPEN) return Commands.NOOP;
            return new Sequential(
                    new Instant(this::setOpen),
                    new Wait(250)
            );
        });
    }

    public ICommand close() {
        return new Lazy(() -> {
            if (gateState == GateState.CLOSE) return Commands.NOOP;
            return new Sequential(
                    new Instant(this::setClose),
                    new Wait(250)
            );
        });
    }

    public void setOpen() {
        setPosition(OPEN);
        gateState = GateState.OPEN;
    }

    public void setClose() {
        setPosition(CLOSE);
        gateState = GateState.CLOSE;
    }
}
