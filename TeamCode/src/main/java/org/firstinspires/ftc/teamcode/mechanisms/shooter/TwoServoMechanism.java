package org.firstinspires.ftc.teamcode.mechanisms.shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.hardware.HwServo;


public class TwoServoMechanism {
    private final HwServo servo1;
    private final HwServo servo2;
    private double servo1Offset;
    private double servo2Offset;
    public TwoServoMechanism(HardwareMap hardwareMap,String id1, String id2){
        servo1 = new HwServo(hardwareMap,id1);
        servo2 = new HwServo(hardwareMap,id2);
    }

    public TwoServoMechanism(HardwareMap hardwareMap,String id1, String id2,double servo1Offset, double servo2Offset){
        servo1 = new HwServo(hardwareMap,id1);
        servo2 = new HwServo(hardwareMap,id2);
        setOffsets(servo1Offset, servo2Offset);
    }
    public void setOffsets(double servo1Offset, double servo2Offset){
        this.servo1Offset = servo1Offset;
        this.servo2Offset = servo2Offset;
    }

    public boolean setPosition(double position){
        boolean servo1Valid = servo1.setPosition(position+servo1Offset);
        boolean servo2Valid = servo2.setPosition(position+servo2Offset);
        return servo1Valid && servo2Valid;
    }

    public double getPosition(){
        return servo1.getPosition()-servo1Offset;
    }

    public void update(){
        servo1.update();
        servo2.update();
    }
}
