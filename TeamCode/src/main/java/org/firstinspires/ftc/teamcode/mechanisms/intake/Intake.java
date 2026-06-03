package org.firstinspires.ftc.teamcode.mechanisms.intake;

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

    public void intakeSlow(){
        intakeMotor.intakeSlow();
    }

    public void outtake(){
        intakeMotor.outtake();
    }

    public void outtakeSlow(){
        intakeMotor.outtakeSlow();
    }

    public void stopIntake(){
        intakeMotor.stop();
    }

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

    public boolean ballInTransfer(){
        return getStates()[0];
    }

    public void deenergize(){
        intakeMotor.deenergize();
    }
}
