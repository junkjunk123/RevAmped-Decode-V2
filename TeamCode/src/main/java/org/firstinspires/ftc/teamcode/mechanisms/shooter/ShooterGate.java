package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commands.Commands;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

public class ShooterGate extends HwServo {
    public static float GATE_OPEN;
    public static float GATE_CLOSE;
    public static int GATE_MOVEMENT_TIME;
    public enum GateState{
        CLOSED,
        OPEN
    }


    private GateState gateState = GateState.CLOSED;
    public ShooterGate(HardwareMap hwMap) {
        super(hwMap, "gate");
    }

    public void init(){
        setPosition(GATE_CLOSE);
    }

    public ICommand close(){
        return new Conditional(() -> gateState != GateState.CLOSED,
                new Sequential(
                        new Instant(() -> {
                            setPosition(GATE_CLOSE);
                            gateState = GateState.CLOSED;
                        }),
                        new Wait(GATE_MOVEMENT_TIME)
                ),
                Commands.NOOP);
    }

    public ICommand open(){
        return new Conditional(() -> gateState != GateState.OPEN,
                new Sequential(
                        new Instant(() -> {
                            setPosition(GATE_OPEN);
                            gateState = GateState.OPEN;
                        }),
                        new Wait(GATE_MOVEMENT_TIME)
                ),
                Commands.NOOP);
    }

    public void setGateOpen(){
        setPosition(GATE_OPEN);
        gateState = GateState.OPEN;
    }


    public String getState(){
        return gateState.name();
    }
}
