package org.firstinspires.ftc.teamcode.utils.math.projectile;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utils.Globals;

@Config
public class HoodInverseKinematics {
    private double distance;
    public static int arc = 1; //change to -1 for a flatter shot
    public static int height = 33; //46in for the goal - 13in for the robot
    public static double startingAngle = Math.PI/6; //rad
    private final double gravity = 386.09;
    private double velocity;
    private double angle;
    private double hoodAngle;
    private double A;

    //at 1260 ticks/sec
    double ticksPerInch = 6.8478;
    double inchesPerTick = 0.14603;

    public HoodInverseKinematics() {}

    public void setDistance(double distance){
        //displacement.getMagnitude()
        this.distance = distance;
    }

    public void setArc(boolean flat){
        if (flat){
            arc = -1;
        } else{
            arc = 1;
        }
    }

    public void setFlywheelVelocity(double InchesPerSecond){
        velocity = InchesPerSecond;
    }

    public void calculateHoodAngle(){
        Globals.telemetry.addData("distance",distance);
        Globals.telemetry.addData("velocity",velocity);
        A = (gravity*distance*distance)/(2*velocity*velocity);
        angle = Math.atan((distance+(arc*Math.sqrt((distance*distance)-(4*A*(height+A)))))/(2*A));
        Globals.telemetry.addData("calc angle",angle);
        hoodAngle = Math.PI- AuraShooterMath.shootingAngleRelativeToHood-angle;
        hoodAngle -= startingAngle;
        Globals.telemetry.addData("hood angle",hoodAngle);
        Globals.telemetry.update();
    }

    public double getAngle(){
        if (!Double.isNaN(hoodAngle)) return hoodAngle;
        else return 0;
    }
}
