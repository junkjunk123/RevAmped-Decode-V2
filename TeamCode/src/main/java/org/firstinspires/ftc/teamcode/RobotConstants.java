package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.intake.ColorManager;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeGate;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeTilt;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Popper;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Splitter;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Table;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.DecodeColorSensor;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.FeederWheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.utils.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.Factory;
import org.firstinspires.ftc.teamcode.utils.Globals;

import java.util.function.Function;

public class RobotConstants {
    public void build() {
        //IntakeMotor Constants
        IntakeMotor.INTAKE = 1.0f; IntakeMotor.INTAKE_SLOW = 0.65f; IntakeMotor.OUTTAKE = -1.0f; IntakeMotor.INTAKE_GATE = 1.0f;
        IntakeMotor.INTAKE_PRELOADS = 0.8f; IntakeMotor.STOPPED = 0.0f; IntakeMotor.SHOOTING = 0.4f; IntakeMotor.OUTTAKE_SLOW = -0.4f;

        //Popper Constants
        Popper.POP = 213/255f; Popper.NEUTRAL = 133/255f; Popper.BLOCK = 195/255f;

        FeederWheel.TARGET_VEL = 2500;

        //Flywheel Constants
        Flywheel.NEAR_VELOCITY = 680; Flywheel.MEDIUM_VELOCITY = 770; Flywheel.FAR_VELOCITY = 1000; Flywheel.CLOSE_AUTO_VELOCITY = 630;
        Flywheel.CORNER_VELOCITY = 880;
        Flywheel.UNSORTED_AUTO_VELOCITY = 730;
        Flywheel.COUNTS_PER_REVOLUTION = 43; Flywheel.RADIUS = 4.094;

        //Hood Constants
        Hood.REST = 51/255f; Hood.HOOD_MAX_POS = 0f; Hood.HOOD_MIN_POS = 0f; Hood.HOOD_MAX_RAD = 0f; Hood.HOOD_MIN_RAD = 0f;

        //HOOD POSITIONS FOR TELEOP
        Hood.FAR_PRESET = 129/255f; Hood.NEAR_PRESET = 51/255f; Hood.MEDIUM_PRESET = 97/255f; Hood.CORNER_PRESET = 115/255f; Hood.HOOD_FAR_COMP = 57/255f;
        Hood.UNSORTED_AUTO = Hood.MEDIUM_PRESET + Math.signum(Hood.NEAR_PRESET - Hood.MEDIUM_PRESET) * 9.5f/255f;

        //Turret Constants
        ServoTurret.REST = 132/255f;

        //When calibrating blue
        Function<Float, Float> turretPos = f -> Globals.allianceColor == AllianceColor.Red ? 2 * ServoTurret.REST - f : f;

        //When calibrating red
        Function<Float, Float> turretPosInv = f -> Globals.allianceColor == AllianceColor.Blue ? 2 * ServoTurret.REST - f : f;
        ServoTurret.FULL_ROTATION = 292/255f; ServoTurret.MS_PER_REVOLUTION = 1080;
        ServoTurret.LEFT_TICKS_LIMIT = 5/255f; ServoTurret.RIGHT_TICKS_LIMIT = 250/255f;
        ServoTurret.FAR_PRESET_RED = 36/255f;
        //done
        ServoTurret.FIFTEEN_OBELISK_DETECTION = turretPos.apply(200/255f);
        //preloads
        ServoTurret.FIFTEEN_BALL_PRELOADS = turretPos.apply(118/255f);
        ServoTurret.UNSORTED_FINAL = turretPos.apply(43/255f);
        ServoTurret.AUTO_SET_1 = turretPos.apply(59/255f);
        ServoTurret.AUTO_SET_2 = turretPos.apply(51/255f);
        ServoTurret.AUTO_SET_3 = turretPos.apply(33/255f);

        ServoTurret.UNSORTED_SET_1 = turretPos.apply(43/255f);
        ServoTurret.UNSORTED_SET_2 = turretPos.apply(42/255f);
        ServoTurret.UNSORTED_SET_3 = turretPos.apply(5/255f);
        ServoTurret.UNSORTED_SET_4 = turretPos.apply(5/255f);
        ServoTurret.UNSORTED_SET_5 = turretPos.apply(5/255f);

        ServoTurret.EIGHTEEN_PRELOADS = turretPosInv.apply(216/255f);
        ServoTurret.EIGHTEEN_FIRST_SET = turretPosInv.apply(252/255f);
        ServoTurret.EIGHTEEN_DETECTION = ServoTurret.REST;
        ServoTurret.EIGHTEEN_SECOND_SET = turretPosInv.apply(200/255f);
        ServoTurret.EIGHTEEN_GATE_SHOOT = turretPosInv.apply(190/255f);
        ServoTurret.EIGHTEEN_THIRD_SET = turretPosInv.apply(202/255f);
        ServoTurret.EIGHTEEN_FOURTH_SET = turretPosInv.apply(192/255f);

        //gate
        IntakeGate.OPEN = 232/255f; IntakeGate.CLOSE = 99/255f;

        //splitter
        Splitter.ACTIVATED = 169/255f; Splitter.NEUTRAL = 57/255f;

        //tilt
        IntakeTilt.TRANSFER = 93/255f; IntakeTilt.INTAKE = 152/255f;

        //updated
        // each compartment is ~32 ticks
        float TABLE_BALL_0 = 244/255f, TABLE_BALL_1 = 212/255f, TABLE_BALL_2 = 180/255f, TABLE_BALL_0_END = 70/255f, TABLE_BALL_1_END = 38/255f, TABLE_BALL_2_END = 6/255f, FULL_REVOLUTION_TICKS = -96/255f;
        Table.setValues(TABLE_BALL_0, TABLE_BALL_1, TABLE_BALL_2, TABLE_BALL_0_END, TABLE_BALL_1_END, TABLE_BALL_2_END,FULL_REVOLUTION_TICKS);
        Table.SHOOT_INCREMENT = -29/255f;

        //ColorSensor Constants
        ColorManager.maxGreenDistanceOne = 60.0f; ColorManager.maxPurpleDistanceOne = 50.0f;
        ColorManager.maxGreenDistanceTwo = 60.0f; ColorManager.maxPurpleDistanceTwo = 60.0f;

        ColorManager.maxPurpleHueOne = 240.0f; ColorManager.minPurpleHueOne = 200.0f;
        ColorManager.maxPurpleHueTwo = 235.0f; ColorManager.minPurpleHueTwo = 195.0f;
        ColorManager.maxGreenHueOne = 165.0f; ColorManager.minGreenHueOne = 155.0f;
        ColorManager.maxGreenHueTwo = 165.0f; ColorManager.minGreenHueTwo = 155.0f;

        //ColorSensor Constants
        DecodeColorSensor.DISTANCE_MAX = 40.0f;
        DecodeColorSensor.GREEN_MAX = 170.0f; DecodeColorSensor.GREEN_MIN = 150.0f;
        DecodeColorSensor.PURPLE_MAX = 250.0f; DecodeColorSensor.PURPLE_MIN = 200.0f;
    }
}
