package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.pedropathing.math.Matrix;

import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeArtifactDetector;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistanceSensors;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.lift.Lift;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.FeederWheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretMTI;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ShooterGate;
import org.firstinspires.ftc.teamcode.mechanisms.vision.DecodeBlobCamera;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MTITele;
import org.firstinspires.ftc.teamcode.utils.data.ListMap;
import org.firstinspires.ftc.teamcode.utils.data.TurretCalibration;
import org.firstinspires.ftc.teamcode.utils.hardware.BlobProcessor;
import org.firstinspires.ftc.teamcode.utils.math.projectile.FarTrackingMath;
import org.firstinspires.ftc.teamcode.utils.vision.BlobTransformer;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

public class RobotConstants {
    public void build() {
        //FAR SHOOTING HOOD & VELOCITY NOT CALIBRATED YET
        IntakeDistanceSensors.useSensors = true;
        IntakeDistanceSensors.INTAKE_SENSOR_DELAY = 200;

        ShooterGate.GATE_OPEN = 121/255f;
        ShooterGate.GATE_CLOSE = 202/255f;
        ShooterGate.GATE_MOVEMENT_TIME = 250;
        Robot.SHOOT_TIME = 600;
        Robot.SHOOT_TIME_FAR = 700;

        //IntakeMotor Constants
        IntakeMotor.INTAKE = 1.0f; IntakeMotor.OUTTAKE = -1.0f; IntakeMotor.STOPPED = 0.0f; IntakeMotor.TRANSFER_FAR = 0.5f;

        FeederWheel.TARGET_VEL = 2500; FeederWheel.INTAKE_POWER = 1.0f; FeederWheel.INTAKE_NO_SENSORS = 0.2f; FeederWheel.TRANSFER_FAR = 0.5f;

        //Flywheel Constants
        Flywheel.NEAR_VELOCITY = 900; Flywheel.MEDIUM_VELOCITY = 940; Flywheel.FAR_VELOCITY = 1200;
        Flywheel.CORNER_VELOCITY = 1050;
        Flywheel.OUTTAKE_POWER = 500;
        Flywheel.COUNTS_PER_REVOLUTION = 43; Flywheel.RADIUS = 4.094;

        //Hood Constants
        Hood.REST = 51/255f; Hood.HOOD_MAX_POS = 0f; Hood.HOOD_MIN_POS = 0f; Hood.HOOD_MAX_RAD = 0f; Hood.HOOD_MIN_RAD = 0f;

        //HOOD POSITIONS FOR TELEOP
        Hood.FAR_PRESET = 128/255f; Hood.NEAR_PRESET = 60/255f; Hood.MEDIUM_PRESET = 75/255f; Hood.CORNER_PRESET = 95/255f; Hood.HOOD_FAR_COMP = 57/255f; //hood far comp not changed

        //Turret Constants - = left + = right
        ServoTurret.REST = 127/255f;

        ServoTurret.FULL_ROTATION = 292/255f; ServoTurret.MS_PER_REVOLUTION = 1080;
        ServoTurret.LEFT_TICKS_LIMIT = 0/255f; ServoTurret.RIGHT_TICKS_LIMIT = 255/255f;
        ServoTurret.FAR_PRESET = TurretCalibration.fromRed(215/255d);

        ServoTurretMTI.REST = 127/255f;

        ServoTurretMTI.FULL_ROTATION = 282/255f; ServoTurretMTI.MS_PER_REVOLUTION = 1080;
        ServoTurretMTI.LEFT_TICKS_LIMIT = 250/255f; ServoTurretMTI.RIGHT_TICKS_LIMIT = 5/255f;

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


        IntakeArtifactDetector.detectionPeriod = 50.0;
    }
}
