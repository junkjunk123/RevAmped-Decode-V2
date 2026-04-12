package org.firstinspires.ftc.teamcode.utils.math.projectile;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.data.MapBuilder;
import org.firstinspires.ftc.teamcode.utils.math.MathUtil;
import org.firstinspires.ftc.teamcode.utils.math.projectile.TrackState.Track;

import java.util.EnumMap;
import java.util.function.BiConsumer;

@Config
public class FarTrackingMath {
    private final Localizer pinpoint;

    private final EnumMap<TrackState, TrackState.Track> heatMap;

    //alliance color is the goal you calibrated on, the code automatically mirrors as needed
    public static Track FAR_1;
    public static Track FAR_2;
    public static Track FAR_3;
    public static Track FAR_4;

    public static Track CLOSE_1;
    public static Track CLOSE_2;
    public static Track CLOSE_3;
    public static Track CLOSE_4;

    private Track target;

    public FarTrackingMath(Localizer pinpoint) {
        this.pinpoint = pinpoint;
        target = new Track(Hood.NEAR_PRESET, 0, ServoTurret.REST);

        FAR_1 = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY - 10,
                207/255d, AllianceColor.Red);
        FAR_2 = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY - 20,
                218/255d, AllianceColor.Red);
        FAR_3 = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY,
                213/255d, AllianceColor.Red);
        FAR_4 = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY + 30,
                222/255d, AllianceColor.Red);

        CLOSE_1 = trackCalibration(233/255d, AllianceColor.Red);
        CLOSE_2 = trackCalibration(250/255d, AllianceColor.Red);
        CLOSE_3 = trackCalibration(-26/255d, AllianceColor.Red);
        CLOSE_4 = trackCalibration(-14/255d, AllianceColor.Red);

        heatMap = MapBuilder.create(() -> new EnumMap<TrackState, Track>(TrackState.class))
                .add(TrackState.FAR_ONE, FAR_1)
                .add(TrackState.FAR_TWO, FAR_2)
                .add(TrackState.FAR_THREE, FAR_3)
                .add(TrackState.FAR_FOUR, FAR_4)
                .add(TrackState.CLOSE_ONE, CLOSE_1)
                .add(TrackState.CLOSE_TWO, CLOSE_2)
                .add(TrackState.CLOSE_THREE, CLOSE_3)
                .add(TrackState.CLOSE_FOUR, CLOSE_4)
                .getMap();
    }

    public static Track trackCalibration(double hoodPos, double flywheelVel, double turretPos, AllianceColor allianceColor) {
        return new Track(hoodPos, flywheelVel,
                allianceColor.equals(AllianceColor.Blue) ? ServoTurret.turretPos.apply(turretPos) : turretPos);
    }

    public static Track trackCalibration(double turretPos, AllianceColor allianceColor) {
        return new Track(0, 0,
                allianceColor.equals(AllianceColor.Blue) ? ServoTurret.turretPos.apply(turretPos) : turretPos);
    }

    public double update() {
        Pose pinpointPose = pinpoint.getPose();
        Globals.telemetry.addData("pinpointPose", pinpointPose);

        /* idt this is needed but its possible? for CLOSE_3 and CLOSE_4
        double targ = target.getTurretPosFromRedCalibration();
        double candidate = targ + ServoTurret.FULL_ROTATION;
        double normalized = Math.abs(targ - ServoTurret.REST) < Math.abs(candidate - ServoTurret.REST) ? targ : candidate;
        return Range.clip(ServoTurret.radToTicks(MathUtil.normalizeAnglePi(
                ServoTurret.ticksToRad(normalized) + pinpointPose.getHeading())
        ), 0, 1);
         */

        return Range.clip(ServoTurret.radToTicks(MathUtil.normalizeAnglePi(
                ServoTurret.ticksToRad(target.getTurretPosFromRedCalibration()) + pinpointPose.getHeading())
        ), 0, 1);
    }

    public BiConsumer<Flywheel, Hood> setState(TrackState state) {
        return (f, h) -> {
            this.target = heatMap.get(state);
            if (target == null) throw new IllegalArgumentException();

            if (state.isFar()) {
                //f.setVelocity(target.flywheelVel());
                //h.setPosition(target.hoodPos());
            }
        };
    }
}
