package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
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

    public static RandomizationState randomizationState;

    public static void setAllianceColor(AllianceColor allianceColor) {
        Globals.allianceColor = allianceColor;

        if (allianceColor.equals(AllianceColor.Red) && Turret.AUTO_PRELOADS > 0) {
            Turret.AUTO_PRELOADS *= -1;
            Turret.AUTO_SET_1 *= -1;
            Turret.AUTO_SET_2 *= -1;
            Turret.AUTO_SET_3 *= -1;
            Turret.FAR_AUTO *= -1;
            Turret.UNSORTED_FINAL *= -1;
            Turret.UNSORTED_GATE *= -1;
            Turret.UNSORTED_AUTO_PRELOADS *= -1;
        }
    }
}
