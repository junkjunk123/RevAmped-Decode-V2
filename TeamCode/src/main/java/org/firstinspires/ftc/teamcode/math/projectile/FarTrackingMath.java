package org.firstinspires.ftc.teamcode.math.projectile;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.utils.Globals;

public class FarTrackingMath {
    private final Localizer pinpoint;

    //all red
    public static float TURRET_PRESET_FORWARD = 220/255f;
    public static double TURRET_PRESET_BACKWARD;
    public static double TURRET_PRESET_RIGHT;
    public static double TURRET_PRESET_LEFT;

    private double turretTarg;

    public FarTrackingMath(Localizer pinpoint) {
        this.pinpoint = pinpoint;
    }

    public double update() {
        Pose pinpointPose = pinpoint.getPose();
        Globals.telemetry.addData("pinpointPose", pinpointPose);
        return ServoTurret.radToTicks(ServoTurret.ticksToRad(turretTarg) + pinpointPose.getHeading());
    }

    public void setTarget(float target) {
        turretTarg = ServoTurret.turretPosInv.apply(target);
    }
}
