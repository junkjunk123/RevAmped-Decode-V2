package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.mechanisms.intake.ColorManager;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;

public class RobotConstants {
    public void build() {
        //IntakeMotor Constants
        IntakeMotor.INTAKE = 1.0f; IntakeMotor.INTAKE_SLOW = 0.65f; IntakeMotor.OUTTAKE = -1.0f; IntakeMotor.INTAKE_GATE = 1.0f;
        IntakeMotor.INTAKE_PRELOADS = 0.8f; IntakeMotor.STOPPED = 0.0f; IntakeMotor.SHOOTING = 0.4f; IntakeMotor.OUTTAKE_SLOW = -0.4f;

        //Popper Constants
        Popper.POP = 123/255f; Popper.NEUTRAL = 35/255f; Popper.BLOCK = 111/255f;

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
        Turret.P = 0.009; Turret.F = 0.01; Turret.P_SECONDARY = 0.003; Turret.F_SECONDARY = 0.1; Turret.PIDF_SWITCH = 50;
        Turret.P_RESET = 0.0022; Turret.F_RESET = 0.09;
        Turret.AUTO_PRELOADS = 520; Turret.AUTO_SET_1 = 535; Turret.AUTO_SET_2 = 535; Turret.AUTO_SET_3 = 550; Turret.FAR_AUTO = 580;
        Turret.UNSORTED_AUTO_PRELOADS = 80; Turret.UNSORTED_GATE = 660; Turret.UNSORTED_FINAL = 723;
        Turret.UNSORTED_SET_1 = Turret.UNSORTED_SET_2 = Turret.UNSORTED_SET_3 = Turret.UNSORTED_SET_5 = Turret.UNSORTED_GATE;
        Turret.UNSORTED_SET_4 = Turret.UNSORTED_GATE;

        Turret.TICKS_LIMIT = 810; Turret.RAD_LIMIT = 3.0 * Math.PI / 4.0; Turret.updateFullRotation();
        //Octocanum Constants
        //OctocanumBack.ENGAGED = 162/255f; OctocanumBack.RAISED = 62/255f;
        //OctocanumFront.ENGAGED = 216/255f; OctocanumFront.RAISED = 118/255f;

        //updated
        float TABLE_BALL_0 = 236/255f, TABLE_BALL_1 = 200/255f, TABLE_BALL_2 = 164/255f, TABLE_BALL_0_END = 112/255f, TABLE_BALL_1_END = 76/255f, TABLE_BALL_2_END = 40/255f, FULL_REVOLUTION_TICKS = -106/255f;
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
