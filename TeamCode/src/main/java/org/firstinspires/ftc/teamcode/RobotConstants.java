package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret.turretPos;
import static org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret.turretPosInv;

import android.graphics.Color;

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
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeBlobCamera;
import org.firstinspires.ftc.teamcode.utils.hardware.BlobProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

public class RobotConstants {
    public void build() {
        //IntakeMotor Constants
        IntakeMotor.INTAKE = 1.0f; IntakeMotor.INTAKE_SLOW = 0.65f; IntakeMotor.OUTTAKE = -1.0f; IntakeMotor.INTAKE_GATE = 1.0f;
        IntakeMotor.INTAKE_PRELOADS = 0.8f; IntakeMotor.STOPPED = 0.0f; IntakeMotor.SHOOTING = 0.4f; IntakeMotor.OUTTAKE_SLOW = -0.4f;

        //Popper Constants
        Popper.POP = 217/255f; Popper.NEUTRAL = 133/255f; Popper.BLOCK = 195/255f;

        FeederWheel.TARGET_VEL = 2500;

        //Flywheel Constants
        Flywheel.NEAR_VELOCITY = 680; Flywheel.MEDIUM_VELOCITY = 770; Flywheel.FAR_VELOCITY = 1000; Flywheel.CLOSE_AUTO_VELOCITY = 630;
        Flywheel.CORNER_VELOCITY = 880;
        Flywheel.UNSORTED_AUTO_VELOCITY = 730;
        Flywheel.COUNTS_PER_REVOLUTION = 43; Flywheel.RADIUS = 4.094;

        //Hood Constants
        Hood.REST = 51/255f; Hood.HOOD_MAX_POS = 0f; Hood.HOOD_MIN_POS = 0f; Hood.HOOD_MAX_RAD = 0f; Hood.HOOD_MIN_RAD = 0f;

        //HOOD POSITIONS FOR TELEOP
        Hood.FAR_PRESET = 129/255f; Hood.NEAR_PRESET = 30/255f; Hood.MEDIUM_PRESET = 65/255f; Hood.CORNER_PRESET = 115/255f; Hood.HOOD_FAR_COMP = 57/255f;
        Hood.UNSORTED_AUTO = Hood.MEDIUM_PRESET + Math.signum(Hood.NEAR_PRESET - Hood.MEDIUM_PRESET) * 9.5f/255f;

        //Turret Constants - = left + = right
        ServoTurret.REST = 129/255f;

        ServoTurret.FULL_ROTATION = 292/255f; ServoTurret.MS_PER_REVOLUTION = 1080;
        ServoTurret.LEFT_TICKS_LIMIT = 5/255f; ServoTurret.RIGHT_TICKS_LIMIT = 250/255f;
        ServoTurret.FAR_PRESET = turretPosInv.apply(217/255d);

        ServoTurret.UNSORTED_AUTO_PRELOADS = turretPosInv.apply(138/255d);
        ServoTurret.UNSORTED_SET_1 = turretPos.apply(56/255d);
        ServoTurret.UNSORTED_SET_2 = turretPos.apply(56/255d);
        ServoTurret.UNSORTED_SET_3 = turretPos.apply(56/255d);
        ServoTurret.UNSORTED_SET_4 = turretPos.apply(56/255d);
        ServoTurret.UNSORTED_SET_5 = turretPos.apply(56/255d);
        ServoTurret.UNSORTED_FINAL = turretPos.apply(59/255d);

        ServoTurret.EIGHTEEN_PRELOADS = turretPosInv.apply(212/255d);
        ServoTurret.EIGHTEEN_FIRST_SET = turretPosInv.apply(249/255d);
        ServoTurret.EIGHTEEN_DETECTION = turretPosInv.apply(193/255d);
        ServoTurret.EIGHTEEN_SECOND_SET = turretPosInv.apply(194/255d);
        ServoTurret.EIGHTEEN_GATE_SHOOT = turretPosInv.apply(188/255d);
        ServoTurret.EIGHTEEN_THIRD_SET = turretPosInv.apply(196/255d);
        ServoTurret.EIGHTEEN_FIFTH_SET = turretPosInv.apply(218/255d);

        //gate
        IntakeGate.OPEN = 232/255f; IntakeGate.CLOSE = 99/255f;

        //splitter
        Splitter.ACTIVATED = 169/255f; Splitter.NEUTRAL = 57/255f;

        //tilt
        IntakeTilt.TRANSFER = 107/255f; IntakeTilt.INTAKE = 163/255f;

        //updated
        // each compartment is ~32 ticks
        float TABLE_BALL_0 = 247/255f, TABLE_BALL_1 = 215/255f, TABLE_BALL_2 = 185/255f, TABLE_BALL_0_END = 72/255f, TABLE_BALL_1_END = 40/255f, TABLE_BALL_2_END = 8/255f, FULL_REVOLUTION_TICKS = -96/255f;
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
        DecodeColorSensor.DISTANCE_MAX = 60.0f;
        DecodeColorSensor.GREEN_MAX = 170.0f; DecodeColorSensor.GREEN_MIN = 150.0f;
        DecodeColorSensor.PURPLE_MAX = 250.0f; DecodeColorSensor.PURPLE_MIN = 200.0f;

        BlobProcessor.debug = true;
        BlobProcessor.blur = 15; BlobProcessor.erode = 30; BlobProcessor.dilate = 30;
        BlobProcessor.morphOperation = ColorBlobLocatorProcessor.MorphOperationType.CLOSING;
        BlobProcessor.boxFitColor = Color.rgb(255,120,31);
        BlobProcessor.circleFitColor = 0;
        BlobProcessor.roiColor = Color.rgb(255,255,255);
        BlobProcessor.contourColor = Color.rgb(3,227,252);

        DecodeBlobCamera.resWidth = 1920;
        DecodeBlobCamera.resHeight = 1080;
    }
}
