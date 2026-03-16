package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.intake.ColorManager;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;

public class RobotConstants {
    public void build() {
        //IntakeMotor Constants
        IntakeMotor.INTAKE = 1.0f; IntakeMotor.INTAKE_SLOW = 0.65f; IntakeMotor.OUTTAKE = -1.0f; IntakeMotor.INTAKE_GATE = 1.0f;
        IntakeMotor.INTAKE_PRELOADS = 0.8f; IntakeMotor.STOPPED = 0.0f; IntakeMotor.SHOOTING = 0.4f; IntakeMotor.OUTTAKE_SLOW = -0.4f;

        //Popper Constants
        Popper.POP = 123/255f; Popper.NEUTRAL = 27/255f; Popper.BLOCK = 111/255f;

        //Flywheel Constants
        Flywheel.NEAR_VELOCITY = 640; Flywheel.MEDIUM_VELOCITY = 780; Flywheel.FAR_VELOCITY = 1080; Flywheel.CLOSE_AUTO_VELOCITY = 630;
        Flywheel.CORNER_VELOCITY = 880;
        Flywheel.UNSORTED_AUTO_VELOCITY = 730;
        Flywheel.COUNTS_PER_REVOLUTION = 43; Flywheel.RADIUS = 4.094;

        //Hood Constants
        Hood.REST = 51/255f; Hood.HOOD_MAX_POS = 0f; Hood.HOOD_MIN_POS = 0f; Hood.HOOD_MAX_RAD = 0f; Hood.HOOD_MIN_RAD = 0f;

        //HOOD POSITIONS FOR TELEOP
        Hood.FAR_PRESET = 127/255f; Hood.NEAR_PRESET = 51/255f; Hood.MEDIUM_PRESET = 97/255f; Hood.CORNER_PRESET = 115/255f;
        Hood.UNSORTED_AUTO = Hood.MEDIUM_PRESET + Math.signum(Hood.NEAR_PRESET - Hood.MEDIUM_PRESET) * 9.5f/255f;

        //Turret Constants
        ServoTurret.FULL_ROTATION = 284;
        ServoTurret.MS_PER_REVOLUTION = 1080;
        ServoTurret.TICKS_LIMIT = 245;
        ServoTurret.REST = 125;
        ServoTurret.RAD_LIMIT = 5.42;
        //Octocanum Constants
        //OctocanumBack.ENGAGED = 162/255f; OctocanumBack.RAISED = 62/255f;
        //OctocanumFront.ENGAGED = 216/255f; OctocanumFront.RAISED = 118/255f;

        //updated
        // each compartment is ~32 ticks
        float TABLE_BALL_0 = 241/255f, TABLE_BALL_1 = 209/255f, TABLE_BALL_2 = 177/255f, TABLE_BALL_0_END = 69/255f, TABLE_BALL_1_END = 37/255f, TABLE_BALL_2_END = 5/255f, FULL_REVOLUTION_TICKS = -145/255f;
        Table.setValues(TABLE_BALL_0, TABLE_BALL_1, TABLE_BALL_2, TABLE_BALL_0_END, TABLE_BALL_1_END, TABLE_BALL_2_END,FULL_REVOLUTION_TICKS);
        Table.SHOOT_INCREMENT = -29/255f;

        //ColorSensor Constants
        ColorManager.maxGreenDistanceOne = 60.0f; ColorManager.maxPurpleDistanceOne = 50.0f;
        ColorManager.maxGreenDistanceTwo = 60.0f; ColorManager.maxPurpleDistanceTwo = 60.0f;
        ColorManager.maxPurpleHueOne = 240.0f; ColorManager.minPurpleHueOne = 200.0f;
        ColorManager.maxPurpleHueTwo = 235.0f; ColorManager.minPurpleHueTwo = 195.0f;
        ColorManager.maxGreenHueOne = 165.0f; ColorManager.minGreenHueOne = 155.0f;
        ColorManager.maxGreenHueTwo = 165.0f; ColorManager.minGreenHueTwo = 155.0f;
    }
}
