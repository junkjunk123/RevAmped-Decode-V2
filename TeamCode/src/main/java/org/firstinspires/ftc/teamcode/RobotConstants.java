package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.pedropathing.math.Matrix;

import org.firstinspires.ftc.teamcode.mechanisms.intake.Intake;
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
import org.firstinspires.ftc.teamcode.opmodes.auto.CloseAuto;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MTITele;
import org.firstinspires.ftc.teamcode.utils.data.ListMap;
import org.firstinspires.ftc.teamcode.utils.data.TurretCalibration;
import org.firstinspires.ftc.teamcode.utils.hardware.BlobProcessor;
import org.firstinspires.ftc.teamcode.utils.math.projectile.FarTrackingMath;
import org.firstinspires.ftc.teamcode.utils.math.projectile.ShooterMath;
import org.firstinspires.ftc.teamcode.utils.math.projectile.SimpleShooterMath;
import org.firstinspires.ftc.teamcode.utils.vision.BlobTransformer;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

public class RobotConstants {
    public void build() {
        //==========THILAN CAN CHANGE THESE==============
        CloseAuto.flywheel_ramp_vel = 2000; //velocity the flywheel ramps in close auto for 30% of the path
        CloseAuto.GATE_WAIT = 3000; //max time in ms the bot waits at the gate
        MTITele.outreach = false; //enable for stop program button (gamepad_2 back)
        ShooterMath.velocityCompensation = false; //SOTM toggle
        IntakeDistanceSensors.useSensors = true; //Distance sensors toggle
        //Intake sensor delays
        IntakeDistanceSensors.INTAKE_SENSOR_DELAY_AUTO = 100;
        IntakeDistanceSensors.INTAKE_SENSOR_DELAY_TELE = 300;

        //Hood comp
        Hood.HOOD_COMP_SOTM_THRESHOLD = 4;
        Hood.HOOD_COMP_SOTM = -15/255f;

        Hood.HOOD_FAR_COMP = -0/255f;
        Hood.HOOD_COMP = -15/255f;
        Hood.HOOD_COMP_DELAY = 150; //delay in ms of how long to wait after start shooting to hood comp


        Robot.SHOOT_TIME = 250; //close shoot time (used in auto)
        Robot.SHOOT_TIME_FAR = 700; //far shoot time (not used rn but prob in auto)
        Robot.CLEANUP_CLOSE_WAIT = 100; //time the bot waits before gate close after release of gamepad_1 right_trigger

        //========================================================

        SimpleShooterMath.ticksPerRad = 311f; //hood angle
        SimpleShooterMath.launchToVel = (double) 900/213; //flywheel velocity

        MTITele.rumbleMS = 200;
        ShooterGate.GATE_OPEN = 121/255f;
        ShooterGate.GATE_CLOSE = 202/255f;
        ShooterGate.GATE_MOVEMENT_TIME = 100;

        Robot.FAR_SHOOT_THRESHOLD_Y = 38;

        //IntakeMotor Constants
        IntakeMotor.INTAKE = 1.0f; IntakeMotor.OUTTAKE = -1.0f; IntakeMotor.STOPPED = 0.0f; IntakeMotor.TRANSFER_FAR = 0.5f;
        IntakeMotor.IDLE_POWER = 0.1f;

        //max vel is 2800
        FeederWheel.TARGET_VEL = 2400; FeederWheel.INTAKE_VELOCITY = 2400; FeederWheel.INTAKE_NO_SENSORS = 480; FeederWheel.TRANSFER_FAR = 1200;

        //Flywheel Constants
        Flywheel.NEAR_VELOCITY = 900; Flywheel.MEDIUM_VELOCITY = 1000; Flywheel.FAR_VELOCITY = 1200;
        Flywheel.CORNER_VELOCITY = 1050;
        Flywheel.OUTTAKE_POWER = 500;
        Flywheel.COUNTS_PER_REVOLUTION = 43; Flywheel.RADIUS = 4.094;
        Flywheel.CLOSE_PRELOADS_VEL = 930;

        //Hood Constants
        Hood.REST = 51/255f; Hood.HOOD_MAX_POS = 0f; Hood.HOOD_MIN_POS = 0f; Hood.HOOD_MAX_RAD = 0f; Hood.HOOD_MIN_RAD = 0f;

        //HOOD POSITIONS FOR TELEOP
        Hood.FAR_PRESET = 128/255f;
        Hood.NEAR_PRESET = 60/255f;
        Hood.MEDIUM_PRESET = 80/255f;
        Hood.CORNER_PRESET = 95/255f;

        //Turret Constants
        ServoTurretMTI.REST = 127/255f;
        ServoTurretMTI.PRELOADS_PRESET = 122/255f;

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
