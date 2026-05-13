package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.pedropathing.math.Matrix;

import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeArtifactDetector;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeTilt;
import org.firstinspires.ftc.teamcode.mechanisms.lift.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.FeederWheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeBlobCamera;
import org.firstinspires.ftc.teamcode.utils.data.ListMap;
import org.firstinspires.ftc.teamcode.utils.data.TurretCalibration;
import org.firstinspires.ftc.teamcode.utils.hardware.BlobProcessor;
import org.firstinspires.ftc.teamcode.utils.math.projectile.FarTrackingMath;
import org.firstinspires.ftc.teamcode.utils.vision.BlobTransformer;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

public class RobotConstants {
    public void build() {
        Robot.SHOOT_TIME = 300;
        //IntakeMotor Constants
        IntakeMotor.INTAKE = 1.0f; IntakeMotor.INTAKE_SLOW = 0.65f; IntakeMotor.OUTTAKE = -1.0f; IntakeMotor.INTAKE_GATE = 1.0f;
        IntakeMotor.INTAKE_PRELOADS = 0.8f; IntakeMotor.STOPPED = 0.0f; IntakeMotor.SHOOTING = 0.4f; IntakeMotor.OUTTAKE_SLOW = -0.4f;

        FeederWheel.TARGET_VEL = 2500;

        //Flywheel Constants
        Flywheel.NEAR_VELOCITY = 680; Flywheel.MEDIUM_VELOCITY = 740; Flywheel.FAR_VELOCITY = 1000; Flywheel.CLOSE_AUTO_VELOCITY = 630;
        Flywheel.CORNER_VELOCITY = 880;
        Flywheel.UNSORTED_AUTO_VELOCITY = 730;
        Flywheel.COUNTS_PER_REVOLUTION = 43; Flywheel.RADIUS = 4.094;

        //Hood Constants
        Hood.REST = 51/255f; Hood.HOOD_MAX_POS = 0f; Hood.HOOD_MIN_POS = 0f; Hood.HOOD_MAX_RAD = 0f; Hood.HOOD_MIN_RAD = 0f;

        //HOOD POSITIONS FOR TELEOP
        Hood.FAR_PRESET = 160/255f; Hood.NEAR_PRESET = 30/255f; Hood.MEDIUM_PRESET = 65/255f; Hood.CORNER_PRESET = 115/255f; Hood.HOOD_FAR_COMP = 57/255f;
        Hood.UNSORTED_AUTO = Hood.MEDIUM_PRESET + Math.signum(Hood.NEAR_PRESET - Hood.MEDIUM_PRESET) * 9.5f/255f;
        Hood.CLOSE_AUTO_FINAL = 3/255f;

        //Turret Constants - = left + = right
        ServoTurret.REST = 127/255f;

        ServoTurret.FULL_ROTATION = 292/255f; ServoTurret.MS_PER_REVOLUTION = 1080;
        ServoTurret.LEFT_TICKS_LIMIT = 0/255f; ServoTurret.RIGHT_TICKS_LIMIT = 255/255f;
        ServoTurret.FAR_PRESET = TurretCalibration.fromRed(215/255d);

        ServoTurret.UNSORTED_AUTO_PRELOADS = TurretCalibration.fromRed(136/255d);
        ServoTurret.UNSORTED_SET_1 = TurretCalibration.fromRed(209/255d);
        ServoTurret.UNSORTED_SET_2 = TurretCalibration.fromRed(206/255d);
        ServoTurret.UNSORTED_SET_3 = TurretCalibration.fromRed(206/255d);
        ServoTurret.UNSORTED_SET_4 = TurretCalibration.fromRed(206/255d);
        ServoTurret.UNSORTED_SET_5 = TurretCalibration.fromRed(206/255d);
        ServoTurret.UNSORTED_FINAL = TurretCalibration.fromRed(199/255d);

        ServoTurret.EIGHTEEN_PRELOADS = TurretCalibration.fromRed(210/255d).withBlue(41/255d);
        ServoTurret.EIGHTEEN_FIRST_SET = TurretCalibration.fromRed(249/255d).withBlue(4/255d);
        ServoTurret.EIGHTEEN_DETECTION = TurretCalibration.fromRed(193/255d).withBlue(63/255d);
        ServoTurret.EIGHTEEN_SECOND_SET = TurretCalibration.fromRed(194/255d).withBlue(59/255d);
        ServoTurret.EIGHTEEN_GATE_SHOOT = TurretCalibration.fromRed(188/255d).withBlue(65/255d);
        ServoTurret.EIGHTEEN_THIRD_SET = TurretCalibration.fromRed(196/255d).withBlue(67/255d);
        ServoTurret.EIGHTEEN_FOURTH_SET = TurretCalibration.fromRed(186/255d).withBlue(68/255d);
        ServoTurret.EIGHTEEN_FIFTH_SET = TurretCalibration.fromRed(218/255d).withBlue(36/255d);

        FarTrackingMath.buildOffsetILUT(
                new ListMap<Double, Double>()
                    .add(0d, 183/255d)
                    .add(Math.PI / 4, 217/255d)
                    .add(Math.PI / 2, 1.0d)
                    .add(Math.PI * 3 / 4.0, 6/255d)
                    .add(Math.PI, 43/255d)
                    .add(Math.PI * 5 / 4.0, 77/255d)
                    .add(Math.PI * 3 / 2.0, 112/255d)
                    .add(Math.PI * 7 / 4.0, 146/255d)
                    .add(Math.PI * 2, 183/255d)
        );


        //tilt
        IntakeTilt.TRANSFER = 126/255f; IntakeTilt.INTAKE = 89/255f; IntakeTilt.GATE_INTAKE = 89/255f;

        Lift.TIME = 1500f;

        IntakeArtifactDetector.detectionPeriod = 50.0;

        BlobProcessor.debug = true;
        BlobProcessor.blur = 15; BlobProcessor.erode = 30; BlobProcessor.dilate = 30;
        BlobProcessor.morphOperation = ColorBlobLocatorProcessor.MorphOperationType.CLOSING;
        BlobProcessor.boxFitColor = Color.rgb(255,120,31);
        BlobProcessor.circleFitColor = 0;
        BlobProcessor.roiColor = Color.rgb(255,255,255);
        BlobProcessor.contourColor = Color.rgb(3,227,252);

        DecodeBlobCamera.resWidth = 1920;
        DecodeBlobCamera.resHeight = 1080;

        BlobTransformer.PHI = Math.toRadians(120);
        BlobTransformer.FOCAL_LENGTH = (DecodeBlobCamera.resWidth / 2.0) / Math.tan(BlobTransformer.PHI / 2);
        BlobTransformer.K = new Matrix(
                new double[][]{
                        {BlobTransformer.FOCAL_LENGTH, 0, DecodeBlobCamera.resWidth / 2.0},
                        {0, BlobTransformer.FOCAL_LENGTH, DecodeBlobCamera.resHeight / 2.0},
                        {0, 0, 1}
                }
        );
        BlobTransformer.K_INV = BlobTransformer.K.inverse();
    }
}
