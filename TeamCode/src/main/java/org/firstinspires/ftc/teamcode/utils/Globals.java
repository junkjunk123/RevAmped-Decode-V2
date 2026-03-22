package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;

/**
 * Globals class to hold global variables and settings for the robot.
 * This class is used to store configuration settings such as alliance color,
 * testing mode, and telemetry instance.
 */

@Config
public final class Globals {
    // Prevent instantiation

    public static double P = 0.8;
    public static double I;
    public static double D;
    public static double F = 0.01;

    private Globals() {
        throw new UnsupportedOperationException("Globals is a utility class and cannot be instantiated");
    }

    public static Telemetry telemetry;
    public static RobotConstants constants = new RobotConstants();
    public static AllianceColor allianceColor = AllianceColor.None;
    public static boolean isTeleOp = true;
    public static double turretStartPos;

    /**
     * Initializes the telemetry instance.
     * This method should be called once at the start of the robot's operation. This is automatically called in a RevAmpedOpMode.
     * to set up the telemetry for logging and debugging.
     *
     * @param telemetry the Telemetry instance to use for logging
     */
    public static void init(Telemetry telemetry) {
        isTeleOp = true;
        constants.build();
        randomizationState = null;
        Globals.telemetry = telemetry;
    }

    public static TrackingThread getTrackingThread() {
        return RobotStateHandler.CycleState.DRIVE_TO_SHOOT.autoTracker;
    }

    public static RandomizationState randomizationState;

    public static void setAllianceColor(AllianceColor allianceColor) {
        Globals.allianceColor = allianceColor;

        if (allianceColor.equals(AllianceColor.Red) && Turret.FAR_AUTO > 0) {
            Turret.AUTO_PRELOADS *= -1;
            Turret.AUTO_SET_1 *= -1;
            Turret.AUTO_SET_2 *= -1;
            Turret.AUTO_SET_3 *= -1;
            Turret.FAR_AUTO *= -1;
            Turret.UNSORTED_FINAL *= -1;
            Turret.UNSORTED_GATE *= -1;
            Turret.UNSORTED_AUTO_PRELOADS *= -1;
            Turret.UNSORTED_SET_1 *= -1;
            Turret.UNSORTED_SET_2 *= -1;
            Turret.UNSORTED_SET_3 *= -1;
            Turret.UNSORTED_SET_4 *= -1;
            Turret.UNSORTED_SET_5 *= -1;
            Turret.FIFTEEN_BALL_PRELOADS *= -1;
            Turret.FIFTEEN_OBELISK_DETECTION = Math.abs(Turret.FIFTEEN_OBELISK_DETECTION);
        } else if (allianceColor.equals(AllianceColor.Blue)) {
            Turret.AUTO_PRELOADS = Math.abs(Turret.AUTO_PRELOADS);
            Turret.AUTO_SET_1 = Turret.AUTO_SET_1 > 0 ? Turret.AUTO_SET_1 : Math.abs(Turret.AUTO_SET_1) - 30;
            Turret.AUTO_SET_2 = Turret.AUTO_SET_2 > 0 ? Turret.AUTO_SET_2 :Math.abs(Turret.AUTO_SET_2) - 30;
            Turret.AUTO_SET_3 = Turret.AUTO_SET_3 > 0 ? Turret.AUTO_SET_3 :Math.abs(Turret.AUTO_SET_3) - 30;
            Turret.FAR_AUTO = Math.abs(Turret.FAR_AUTO);
            Turret.UNSORTED_FINAL = Math.abs(Turret.UNSORTED_FINAL);
            Turret.UNSORTED_GATE = Math.abs(Turret.UNSORTED_GATE);
            Turret.UNSORTED_AUTO_PRELOADS = Math.abs(Turret.UNSORTED_AUTO_PRELOADS);
            Turret.UNSORTED_SET_1 = Math.abs(Turret.UNSORTED_SET_1);
            Turret.UNSORTED_SET_2 = Math.abs(Turret.UNSORTED_SET_2);
            Turret.UNSORTED_SET_3 = Math.abs(Turret.UNSORTED_SET_3);
            Turret.UNSORTED_SET_4 = Math.abs(Turret.UNSORTED_SET_4);
            Turret.UNSORTED_SET_5 = Math.abs(Turret.UNSORTED_SET_5);
            Turret.FIFTEEN_BALL_PRELOADS = Math.abs(Turret.FIFTEEN_BALL_PRELOADS);
            Turret.FIFTEEN_OBELISK_DETECTION = -Math.abs(Turret.FIFTEEN_OBELISK_DETECTION);
        }
    }
}
