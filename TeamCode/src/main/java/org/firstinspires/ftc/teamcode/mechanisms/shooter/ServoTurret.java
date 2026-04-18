package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.commands.Conditional;
import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;

import java.util.function.Function;

@Config
public class ServoTurret extends HwServo {
    //When calibrating blue
    public static Function<Double, Double> turretPos = f -> Globals.allianceColor == AllianceColor.Red ? 2 * ServoTurret.REST - f : f;

    //When calibrating red
    public static Function<Double, Double> turretPosInv = f -> Globals.allianceColor == AllianceColor.Blue ? 2 * ServoTurret.REST - f : f;

    public static double LEFT_TICKS_LIMIT;
    public static double RIGHT_TICKS_LIMIT;
    public static double FULL_ROTATION;
    public static double REST;

    public static double EIGHTEEN_DETECTION;
    public static double EIGHTEEN_PRELOADS;
    public static double EIGHTEEN_FIRST_SET;
    public static double EIGHTEEN_SECOND_SET;
    public static double EIGHTEEN_GATE_SHOOT;
    public static double EIGHTEEN_THIRD_SET;
    public static double EIGHTEEN_FOURTH_SET;
    public static double EIGHTEEN_FIFTH_SET;

    public static double AUTO_PRELOADS;
    public static double AUTO_SET_1;
    public static double AUTO_SET_2;
    public static double AUTO_SET_3;
    public static double FAR_AUTO;
    public static double FIFTEEN_BALL_PRELOADS;
    public static double FIFTEEN_OBELISK_DETECTION;
    public static double UNSORTED_AUTO_PRELOADS; //doesn't matter for 15
    public static double UNSORTED_GATE;
    public static double UNSORTED_SET_1;
    public static double UNSORTED_SET_2;
    public static double UNSORTED_SET_3;
    public static double UNSORTED_SET_4;
    public static double UNSORTED_SET_5;
    public static double UNSORTED_FINAL;
    public static double FAR_PRESET;

    public static double MS_PER_REVOLUTION = 1500;

    public static double ticksPerRotation() {
        return FULL_ROTATION;
    }
    public static double ticksPerDegree() {return FULL_ROTATION / 360.0;}
    public static double ticksPerRad() {return FULL_ROTATION / Math.PI / 2;}
    public static double degreesToTicks(double degrees) {return degrees * ticksPerDegree() + REST;}
    public static double ticksToDegrees(double ticks) {return (ticks - REST) / ticksPerDegree();}
    public static double ticksToRad(double ticks) {return (ticks - REST) / ticksPerRad();}
    public static double radToTicks(double rad) {return rad * ticksPerRad() + REST;}

    private ServoTurretState state = ServoTurretState.PresetState.REST;
    private double offset = 0;

    /**
     * @param hwMap hardwareMap
     */
    public ServoTurret(HardwareMap hwMap) {
        super(hwMap, "turret", "turret2");
    }

    public ICommand setState(ServoTurretState state) {
        return new Conditional(
                //havish PLEASE REMOVE THIS AFTER MTI APP T_T
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

    //PLEASE REMOVE THIS AFTER MTI APPS T_T
    public void manualSOTM(double offset){
        setPosition(this.state.targetPos()-offset);
    }

    public void manual(){
        setPosition(this.state.targetPos());
    }

}
