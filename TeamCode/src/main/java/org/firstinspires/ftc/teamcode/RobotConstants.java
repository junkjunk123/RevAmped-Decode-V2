package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.mechanisms.intake.ColorManager;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.octocanum.OctocanumBack;
import org.firstinspires.ftc.teamcode.mechanisms.octocanum.OctocanumFront;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;

public class RobotConstants {
    public void build() {
        //IntakeMotor Constants
        IntakeMotor.INTAKE = 1.0f; IntakeMotor.INTAKE_SLOW = 0.5f; IntakeMotor.OUTTAKE = -1.0f;
        IntakeMotor.STOPPED = 0.0f; IntakeMotor.SHOOTING = 0.4f;IntakeMotor.OUTTAKE_SLOW = -0.5f;

        //Popper Constants
        Popper.POP = 243/255f; Popper.NEUTRAL = 161/255f;

        //Flywheel Constants
        Flywheel.NEAR_VELOCITY = 725; Flywheel.MEDIUM_VELOCITY = 875; Flywheel.FAR_VELOCITY = 1140; Flywheel.AUTO_VELOCITY = 780;
        Flywheel.COUNTS_PER_REVOLUTION = 43; Flywheel.RADIUS = 4.094;

        //Hood Constants
        Hood.REST = 40/255f; Hood.HOOD_MAX_POS = 0f; Hood.HOOD_MIN_POS = 0f; Hood.HOOD_MAX_RAD = 0f; Hood.HOOD_MIN_RAD = 0f;

        //HOOD POSITIONS FOR TELEOP
        Hood.FAR_PRESET = 130/255f; Hood.NEAR_PRESET = 40/255f; Hood.MEDIUM_PRESET = 130/255f;

        //Turret Constants
        Turret.P = 0.01; Turret.F = 0.01;
        Turret.AUTO_PRELOADS = 438; Turret.AUTO_SET_1 = 513; Turret.AUTO_SET_2 = 513; Turret.AUTO_SET_3 = 563; Turret.FAR_AUTO = 691;
        Turret.TICKS_LIMIT = 817; Turret.RAD_LIMIT = Math.PI; Turret.updateFullRotation();

        //Octocanum Constants
        OctocanumBack.ENGAGED = 162/255f; OctocanumBack.RAISED = 56/255f;
        OctocanumFront.ENGAGED = 216/255f; OctocanumFront.RAISED = 110/255f;

        //Table Constants
        //float TURRET_BALL_0 = 242/255f, TURRET_BALL_1 = 205/255f, TURRET_BALL0_END = 88/255f;
        //Table.setValues(TURRET_BALL_0, TURRET_BALL_1, TURRET_BALL0_END);
        //full rotation 146/255f ball 1 250/255

        float TURRET_BALL_0 = 250/255f, TURRET_BALL_1 = 201/255f, TURRET_BALL_2 = 155/255f, TURRET_BALL_0_END = 100/255f, TURRET_BALL_1_END = 51/255f, TURRET_BALL_2_END = 5, FULL_REVOLUTION_TICKS = 146/255f;
        Table.setValues(TURRET_BALL_0,TURRET_BALL_1,TURRET_BALL_2, TURRET_BALL_0_END, TURRET_BALL_1_END, TURRET_BALL_2_END,FULL_REVOLUTION_TICKS);
        //ColorSensor Constants
        ColorManager.maxGreenDistanceOne = 60.0f; ColorManager.maxPurpleDistanceOne = 50.0f;
        ColorManager.maxGreenDistanceTwo = 60.0f; ColorManager.maxPurpleDistanceTwo = 60.0f;
        ColorManager.maxPurpleHueOne = 240.0f; ColorManager.minPurpleHueOne = 200.0f;
        ColorManager.maxPurpleHueTwo = 235.0f; ColorManager.minPurpleHueTwo = 195.0f;
        ColorManager.maxGreenHueOne = 165.0f; ColorManager.minGreenHueOne = 155.0f;
        ColorManager.maxGreenHueTwo = 165.0f; ColorManager.minGreenHueTwo = 155.0f;
    }
}
