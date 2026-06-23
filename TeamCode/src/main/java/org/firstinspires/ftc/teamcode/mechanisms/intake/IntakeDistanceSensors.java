package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Conditional;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.commands.Wait;
import com.pedropathing.ivy.groups.Sequential;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.Commands;

import java.util.Arrays;

@Config
public class IntakeDistanceSensors {
    public static int[] INTAKE_SENSOR_DELAY_AUTO;
    public static int[] INTAKE_SENSOR_DELAY_TELE;
    public static int INTAKE_SENSOR_DELAY;
    private final IntakeArtifactDetector[] distanceSensors;
    private final boolean[] distanceStates;
    public static boolean useSensors;
    private boolean on;
    public boolean readIntakeDistance = false;
    public boolean waiting = false;
    private int currentSensor;
    public IntakeDistanceSensors(HardwareMap hardwareMap){
        distanceSensors = new IntakeArtifactDetector[] {
                new IntakeArtifactDetector(hardwareMap,"ball1"),
                new IntakeArtifactDetector(hardwareMap,"ball2"),
                new IntakeArtifactDetector(hardwareMap,"ball3")};
        distanceStates = new boolean[] {false,false,false};
        start();
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
            currentSensor = 0;
            on = true;
            distanceSensors[0].start();
            distanceSensors[1].start();
            distanceSensors[2].start();
        }
    }

    public void updateSensors(boolean checkFalse) {
//        for (int i = 0; i < 3; i++) {
//            if ((i < 2) || (readIntakeDistance)) {
//                if (!distanceStates[i]) {
//                    distanceSensors[i].update();
//                    distanceStates[i] = distanceSensors[i].getReading();
//                }
//            }
//        }
        if (currentSensor <= 2 && !distanceStates[currentSensor]) {
            distanceSensors[currentSensor].update();
            if (distanceSensors[currentSensor].getReading()) {
                distanceStates[currentSensor] = true;
                Scheduler.getInstance().schedule(
                    new Sequential(
                        new Conditional(
                            () -> Globals.isTeleOp,
                            new Wait(INTAKE_SENSOR_DELAY_TELE[currentSensor]),
                            new Wait(INTAKE_SENSOR_DELAY_AUTO[currentSensor])
                        ),
                        new Instant(() -> currentSensor++)
                    )
                );

            }
        }
    }

    public boolean[] getStates(){
        return distanceStates;
    }

    public boolean[] getSensorStates(){
        //just directly gets the sensor state
        distanceSensors[0].update();
        distanceSensors[1].update();
        distanceSensors[2].update();
        return new boolean[] {distanceSensors[0].getReading(),distanceSensors[1].getReading(),distanceSensors[2].getReading()};

    }

    public int getCurrentSensor(){
        return currentSensor;
    }

    public void setOn (boolean on){
        this.on = on;
    }

    public boolean isOn() {
        return on && useSensors;
    }

    public void clear(){
        Arrays.fill(distanceStates,false);
        waiting = false;
        readIntakeDistance = false;
        currentSensor = 0;
    }

    public void update() {
        update(true);
    }

    public void update(boolean checkFalse) {
        if (isOn()){
            updateSensors(checkFalse);
        }
    }
}
