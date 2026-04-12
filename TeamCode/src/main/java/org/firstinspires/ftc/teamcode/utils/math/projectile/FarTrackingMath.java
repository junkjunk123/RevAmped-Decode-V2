package org.firstinspires.ftc.teamcode.utils.math.projectile;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.data.MapBuilder;
import org.firstinspires.ftc.teamcode.utils.math.projectile.TrackState.Track;

import java.util.EnumMap;
import java.util.function.BiConsumer;

@Config
public class FarTrackingMath {
    private final Localizer pinpoint;

    private final EnumMap<TrackState, TrackState.Track> heatMap;

    //alliance color is the goal you calibrated on, the code automatically mirrors as needed
    public static Track FAR_1 = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY, 220/255f, AllianceColor.Red);

    private Track target;

    public FarTrackingMath(Localizer pinpoint) {
        this.pinpoint = pinpoint;
        target = new Track(Hood.NEAR_PRESET, 0, ServoTurret.REST);

        heatMap = MapBuilder.create(() -> new EnumMap<TrackState, Track>(TrackState.class))
                .add(TrackState.FAR_ONE, FAR_1)
                .getMap();
    }

    public static Track trackCalibration(double hoodPos, double flywheelVel, double turretPos, AllianceColor allianceColor) {
        return new Track(hoodPos, flywheelVel,
                allianceColor.equals(AllianceColor.Blue) ? ServoTurret.turretPos.apply(turretPos) : turretPos);
    }

    public double update() {
        Pose pinpointPose = pinpoint.getPose();
        Globals.telemetry.addData("pinpointPose", pinpointPose);
        return ServoTurret.radToTicks(ServoTurret.ticksToRad(target.getTurretPosFromRedCalibration()) + pinpointPose.getHeading());
    }

    public BiConsumer<Flywheel, Hood> setState(TrackState state) {
        return (f, h) -> {
            this.target = heatMap.get(state);
            if (target == null) throw new IllegalArgumentException();

            if (state.isFar()) {
                f.setVelocity(target.flywheelVel());
                h.setPosition(target.hoodPos());
            }
        };
    }
}
