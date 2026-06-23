package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeArtifactDetector;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeDistanceSensors;
import org.firstinspires.ftc.teamcode.mechanisms.intake.IntakeMotor;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.FeederWheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurretMTI;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ShooterGate;
import org.firstinspires.ftc.teamcode.opmodes.auto.CloseSideSpikeAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.Close24Auto;
import org.firstinspires.ftc.teamcode.opmodes.auto.FarAuto;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MTITele;
import org.firstinspires.ftc.teamcode.utils.data.ListMap;
import org.firstinspires.ftc.teamcode.utils.math.projectile.FarTrackingMath;
import org.firstinspires.ftc.teamcode.utils.math.projectile.ShooterMath;
import org.firstinspires.ftc.teamcode.utils.math.projectile.SimpleShooterMath;

public class RobotConstants {
    public void build() {
        //==========THILAN CAN CHANGE THESE==============
        Close24Auto.flywheel_ramp_vel = 2000; //velocity the flywheel ramps in close auto for 30% of the path
        Close24Auto.GATE_WAIT = 3000; //max time in ms the bot waits at the gate
        CloseSideSpikeAuto.flywheel_ramp_vel = 2000; //velocity the flywheel ramps in close auto for 30% of the path
        CloseSideSpikeAuto.GATE_WAIT = 3000; //max time in ms the bot waits at the gate
        FarAuto.FLYWHEEL_RAMP_UP_WAIT = 1500;
        FarAuto.SHOOT_DELAY = 150;
        MTITele.outreach = false; //enable for stop program button (gamepad_2 back)
        MTITele.DRIVER_TURRET_OFFSET = -7/255f;
        MTITele.calibration = false;
        ShooterMath.velocityCompensation = false; //SOTM toggle
        IntakeDistanceSensors.useSensors = true; //Distance sensors toggle
        //Intake sensor delays {between 0&1, between 1&2, 0}
        IntakeDistanceSensors.INTAKE_SENSOR_DELAY_AUTO = new int[] {0,100,0};
        IntakeDistanceSensors.INTAKE_SENSOR_DELAY_TELE = new int[]{0,100,0};
        SimpleShooterMath.tooCloseThreshold = 60;

        //Hood comp
        Hood.HOOD_COMP_SOTM_THRESHOLD = 4;
        Hood.HOOD_COMP_SOTM = -15/255f;
        Hood.HOOD_COMP_SOTM_BACKWARDS = -7/255f;

        Hood.HOOD_FAR_COMP = -0/255f;
        Hood.HOOD_COMP = -25/255f;
        Hood.HOOD_COMP_DELAY = 70; //delay in ms of how long to wait after start shooting to hood comp


        Robot.SHOOT_TIME = 250; //close shoot time (used in auto)
        Robot.SHOOT_TIME_FAR = 500; //far shoot time (not used rn but prob in auto)
        Robot.CLEANUP_CLOSE_WAIT = 100; //time the bot waits before gate close after release of gamepad_1 right_trigger

        //========================================================

        SimpleShooterMath.ticksPerRad = 311f; //hood angle
        SimpleShooterMath.launchToVel = (double) 900/213; //flywheel velocity

        MTITele.rumbleMS = 200;
        ShooterGate.GATE_OPEN = 121/255f;
        ShooterGate.GATE_CLOSE = 202/255f;
        ShooterGate.GATE_MOVEMENT_TIME = 100;

        Robot.FAR_SHOOT_THRESHOLD_Y = 48;

        //IntakeMotor Constants
        IntakeMotor.INTAKE = 1.0f; IntakeMotor.OUTTAKE = -1.0f; IntakeMotor.STOPPED = 0.0f; IntakeMotor.SHOOT_FAR = 0.7f;
        IntakeMotor.IDLE_POWER = 0; IntakeMotor.SHOOT = 1.0f;

        //max vel is 2800
        FeederWheel.TARGET_VEL = 2300; FeederWheel.INTAKE_VELOCITY = 2200; FeederWheel.INTAKE_NO_SENSORS = 480; FeederWheel.SHOOT_FAR = 1200; FeederWheel.SHOOT_VELOCITY = 2300;

        //Flywheel Constants
        Flywheel.NEAR_VELOCITY = 900; Flywheel.MEDIUM_VELOCITY = 1000; Flywheel.FAR_VELOCITY = 1175;
        Flywheel.CORNER_VELOCITY = 1050;
        Flywheel.OUTTAKE_POWER = 500;
        Flywheel.COUNTS_PER_REVOLUTION = 43; Flywheel.RADIUS = 4.094;
        Flywheel.CLOSE_PRELOADS_VEL = 830;

        //Hood Constants
        Hood.REST = 51/255f; Hood.HOOD_MAX_POS = 0f; Hood.HOOD_MIN_POS = 0f; Hood.HOOD_MAX_RAD = 0f; Hood.HOOD_MIN_RAD = 0f;

        //HOOD POSITIONS FOR TELEOP
        Hood.FAR_PRESET = 128/255f;
        Hood.NEAR_PRESET = 60/255f;
        Hood.MEDIUM_PRESET = 80/255f;
        Hood.CORNER_PRESET = 95/255f;

        //Turret Constants
        ServoTurretMTI.REST = 128/255f;
        ServoTurretMTI.RED_CLOSE_PRELOADS_24 = 122/255f;
        ServoTurretMTI.BLUE_CLOSE_PRELOADS_24 = 134/255f;
        ServoTurretMTI.RED_CLOSE_PRELOADS_SIDE_SPIKE = 243/255f;

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
