package org.firstinspires.ftc.teamcode.mechanisms.octocanum;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Octocanum {
    private OctocanumFront front;
    private OctocanumBack back;
    private OctoState octoState;

    public Octocanum(HardwareMap hardwareMap){
        front = new OctocanumFront(hardwareMap);
        back = new OctocanumBack(hardwareMap);
        octoState = OctoState.UP;
        raise();
    }

    public enum OctoState{
        UP,
        DOWN;
    }

    public void engage(){
        front.engage();
        back.engage();
        octoState = OctoState.DOWN;
    }

    public void raise(){
        front.raise();
        back.raise();
        octoState = OctoState.UP;
    }

    public void toggle(){
        switch (octoState){
            case UP -> {engage();}
            case DOWN -> {raise();}
        }
    }

    public void update(){
        front.update();
        back.update();
    }

    public OctoState getState(){
        return octoState;
    }
}
