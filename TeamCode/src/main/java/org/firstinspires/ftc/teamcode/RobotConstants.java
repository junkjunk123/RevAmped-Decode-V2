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
        IntakeMotor.INTAKE = 1.0f; IntakeMotor.INTAKE_SLOW = 0.5f; IntakeMotor.OUTTAKE = -1.0f;
        IntakeMotor.STOPPED = 0.0f; IntakeMotor.SHOOTING = 0.4f;IntakeMotor.OUTTAKE_SLOW = -0.5f;

        //Popper Constants
        Popper.POP = 22/255f; Popper.NEUTRAL = 102/255f;

        //Flywheel Constants
        Flywheel.P = 0.0005; Flywheel.COUNTS_PER_REVOLUTION = 43; Flywheel.RADIUS = 4.094;
        Flywheel.kStatic = 0.05; Flywheel.kV = 0.00045; Flywheel.kA = 0; 
        Flywheel.STATE_STDDEV = 2; Flywheel.MEASUREMENT_STDDEV = 0.2;
        Flywheel.MAX_ACCELERATION = Flywheel.FAR_VELOCITY * Flywheel.FAR_VELOCITY / Flywheel.RADIUS;
        Flywheel.NEAR_VELOCITY = 725; Flywheel.MEDIUM_VELOCITY = 875; Flywheel.FAR_VELOCITY = 1095; Flywheel.AUTO_VELOCITY = 780;

        //Hood Constants
        Hood.REST = 200/255f; Hood.HOOD_MAX_POS = 0f; Hood.HOOD_MIN_POS = 0f; Hood.HOOD_MAX_RAD = 0f; Hood.HOOD_MIN_RAD = 0f;

        //HOOD POSITIONS FOR TELEOP
        Hood.FAR_PRESET = 220/255f; Hood.NEAR_PRESET = 190/255f; Hood.MEDIUM_PRESET = 220/255f;

        //Turret Constants
        Turret.P = 0.01; Turret.F = 0.01;
        Turret.AUTO_PRELOADS = 438; Turret.AUTO_SET_1 = 513; Turret.AUTO_SET_2 = 513; Turret.AUTO_SET_3 = 563; Turret.FAR_AUTO = 691;
        Turret.TICKS_LIMIT = 560; Turret.RAD_LIMIT = Math.PI / 2;

        //Table Constants
        float TURRET_BALL_1 = 242/255f, TURRET_BALL_2 = 205/255f, TURRET_BALL1_END = 88/255f;
        Table.setValues(TURRET_BALL_1, TURRET_BALL_2, TURRET_BALL1_END);

        //ColorSensor Constants
        ColorManager.maxGreenDistanceOne = 60.0f; ColorManager.maxPurpleDistanceOne = 50.0f;
        ColorManager.maxGreenDistanceTwo = 60.0f; ColorManager.maxPurpleDistanceTwo = 60.0f;
        ColorManager.maxPurpleHueOne = 240.0f; ColorManager.minPurpleHueOne = 200.0f;
        ColorManager.maxPurpleHueTwo = 235.0f; ColorManager.minPurpleHueTwo = 195.0f;
        ColorManager.maxGreenHueOne = 165.0f; ColorManager.minGreenHueOne = 155.0f;
        ColorManager.maxGreenHueTwo = 165.0f; ColorManager.minGreenHueTwo = 155.0f;
    }
}
