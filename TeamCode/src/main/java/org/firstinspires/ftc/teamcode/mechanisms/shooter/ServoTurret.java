package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

@Config
public class ServoTurret extends HwServo {
    public static double LEFT_TICKS_LIMIT;
    public static double RIGHT_TICKS_LIMIT;
    public static double FULL_ROTATION;
    public static float REST;

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
    public static double FAR_PRESET_BLUE;
    public static double FAR_PRESET_RED;

    public static double MS_PER_REVOLUTION = 1500;

    public static float ticksPerRotation() {
        return (float) FULL_ROTATION;
    }

    private ServoTurretState state = ServoTurretState.PresetState.REST;

    /**
     * @param hwMap hardwareMap
     */
    public ServoTurret(HardwareMap hwMap) {
        super(hwMap, "turret1", "turret2", "turret3");
    }

    public ICommand setState(ServoTurretState state) {
        return new Conditional(
                () -> setPosition(state.targetPos()),
                new Sequential(
                        new Instant(() -> this.state = state),
                        new Wait(Math.abs(state.targetPos() - getPosition()) / ticksPerRotation() * MS_PER_REVOLUTION)
                ),
                new Instant(() -> this.state = state)
        );
    }

    public ServoTurretState getState() {
        return state;
    }

    public ICommand resetTurret() {
        return setState(ServoTurretState.PresetState.REST);
    }

    public void unsortedAutoSet(int setNum) {
        switch (setNum + 1) {
            case 1 -> setPosition(UNSORTED_SET_1);
            case 2 -> setPosition(UNSORTED_SET_2);
            case 3 -> setPosition(UNSORTED_SET_3);
            case 4 -> setPosition(UNSORTED_SET_4);
            case 5 -> setPosition(UNSORTED_SET_5);
            default -> setPosition(UNSORTED_AUTO_PRELOADS);
        }
    }

    public void move(ServoTurretState state) {
        setPosition(state.targetPos());
        this.state = state;
    }

    public void next() {
        if (state instanceof ServoTurretState.PresetState p)
            move(p.next());
        else
            move(ServoTurretState.PresetState.REST);
    }

    public void previous() {
        if (state instanceof ServoTurretState.PresetState p)
            move(p.previous());
        else
            move(ServoTurretState.PresetState.REST);
    }
}
