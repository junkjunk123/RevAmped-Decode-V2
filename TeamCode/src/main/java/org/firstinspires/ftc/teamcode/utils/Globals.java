package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.mechanisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.TrackingThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Turret;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.commands.RandomizationState;

/**
 * Globals class to hold global variables and settings for the robot.
 * This class is used to store configuration settings such as alliance color,
 * testing mode, and telemetry instance.
 */

@Config
public final class Globals {
    // Prevent instantiation

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

    public static RandomizationState randomizationState;

    public static void setAllianceColor(AllianceColor allianceColor) {
        Globals.allianceColor = allianceColor;
    }
}
