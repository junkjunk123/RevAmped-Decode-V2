package org.firstinspires.ftc.teamcode.mechanisms.intake;

import com.pedropathing.ivy.ICommand;
import com.pedropathing.ivy.commands.Instant;
import com.pedropathing.ivy.groups.Parallel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public final IntakeMotor intakeMotor;
    public final IntakeDistanceSensors distanceSensors;

    public Intake(HardwareMap hardwareMap){
        intakeMotor = new IntakeMotor(hardwareMap);
        distanceSensors = new IntakeDistanceSensors(hardwareMap);
    }

    public void init(){
        stopIntake();
        distanceSensors.start();
    }

    public void intake(){
        intakeMotor.intake();
    }

    public void shootFar(){intakeMotor.shootFar();}
    public void shoot(){intakeMotor.shoot();}

    public void outtake(){
        intakeMotor.outtake();
    }

    public void stopIntake(){
        intakeMotor.stop();
    }

    public void idle(){intakeMotor.idle();}

    public void stopSensors(){
        distanceSensors.stop();
        distanceSensors.clear();
    }

    public void startSensors(){distanceSensors.start();}

    public void update(){
            intakeMotor.update();
            distanceSensors.update();
    }

    public boolean[] getStates(){
        return distanceSensors.getStates();
    }

    public boolean hasThree() {
        boolean[] distanceStates = getStates();
        return distanceStates[0] && distanceStates[1] && distanceStates[2];
    }

    public boolean hasTwo(){
        boolean [] distanceStates = getStates();
        return distanceStates[0] && distanceStates[1];
    }

    public int numBalls() {
        boolean[] distanceStates = getStates();
        int num = 0;
        for (int i = 0; i < 3; i++) {
            if (distanceStates[i]) num++;
        }
        return num;
    }

    public boolean ballInTransfer(){
        return getStates()[0];
    }

    public void deenergize(){
        intakeMotor.deenergize();
    }
}
