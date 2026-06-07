package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Conditional;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.commands.Commands;

import java.util.Arrays;

@Config
public class IntakeDistanceSensors {
    public static int INTAKE_SENSOR_DELAY;
    private final IntakeArtifactDetector[] distanceSensors;
    private final boolean[] distanceStates;
    public static boolean useSensors = false;
    private boolean on;
    private boolean pause = false;
    private boolean readIntakeDistance = false;

    public IntakeDistanceSensors(HardwareMap hardwareMap){
        distanceSensors = new IntakeArtifactDetector[] {
                new IntakeArtifactDetector(hardwareMap,"ball1"),
                new IntakeArtifactDetector(hardwareMap,"ball2"),
                new IntakeArtifactDetector(hardwareMap,"ball3")};
        distanceStates = new boolean[] {false,false,false};
    }

    public void stop(){
        if (useSensors) {
            on = false;
            distanceSensors[0].stop();
            distanceSensors[1].stop();
            distanceSensors[2].stop();
        }
    }

    public void start(){
        if (useSensors) {
            on = true;
            pause = false;
            distanceSensors[0].start();
            distanceSensors[1].start();
            distanceSensors[2].start();
        }
    }

    public void updateSensors(boolean checkFalse){
        int num = 0;
        for (int i = 0; i < 3; i++) {
            if ((i < 2) || (readIntakeDistance)) {
                if (!distanceStates[i]) {
                    distanceSensors[i].update();
                    distanceStates[i] = distanceSensors[i].getReading();
                }
            }
            if (distanceStates[i]) num++;
            if (checkFalse && num == 3) pause = true;
        }
    }

    public boolean[] getStates(){
        return distanceStates;
    }

    public void setOn (boolean on){
        this.on = on;
    }

    public boolean isOn() {
        return on && useSensors;
    }

    public void clear(){
        Arrays.fill(distanceStates,false);
        pause = false;
    }

    public ICommand update() {
        return update(true);
    }

    public ICommand update(boolean checkFalse) {
        if (on && useSensors) {
            return new Sequential(
                new Instant(() -> updateSensors(checkFalse)),
                new Conditional(
                    () -> distanceStates[1] && !readIntakeDistance,
                    new Sequential(
                        new Wait(INTAKE_SENSOR_DELAY),
                        new Instant(() -> readIntakeDistance = true)
                    ),
                    Commands.NOOP)
            );
        }
        return Commands.NOOP;
    }

    public boolean shouldPause() {
        return isOn() && pause;
    }

    public void setPause(boolean pause) {
        this.pause = pause;
    }
}
