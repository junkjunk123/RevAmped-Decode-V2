package org.firstinspires.ftc.teamcode.utils.math.projectile;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.GyroThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.data.MapBuilder;
import org.firstinspires.ftc.teamcode.utils.math.ILUT;
import org.firstinspires.ftc.teamcode.utils.math.MathUtil;
import org.firstinspires.ftc.teamcode.utils.math.projectile.TrackState.Track;

import java.util.EnumMap;
import java.util.function.BiConsumer;

@Config
public class FarTrackingMath {
    private final PinpointLocalizer pinpoint;

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

    public static Track FAR_AUTO;

    public static Track REST;

    public static ILUT offsetInterpol;

    private Track target;

    public static double ANGULAR_CONSTANT = 0.06;

    public FarTrackingMath(Localizer pinpoint) {
        this.pinpoint = (PinpointLocalizer) pinpoint;

        REST = trackCalibration(ServoTurret.REST, AllianceColor.Blue);

        FAR_1 = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY - 15,
                213/255d, 181/255d);
        FAR_2 = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY - 5,
                218/255d, 181/255d);
        FAR_3 = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY + 15,
                228/255d, 174/255d);
        FAR_4 = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY + 25,
                229/255d, 173/255d);

        FAR_AUTO = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY - 25,
                215.25/255d, AllianceColor.Red);

        CLOSE_1 = trackCalibration(233/255d, 166/255d);
        CLOSE_2 = trackCalibration(250/255d, 147/255d);
        CLOSE_3 = trackCalibration(-26/255d, 135/255d);
        CLOSE_4 = trackCalibration(-14/255d, AllianceColor.Red);

        heatMap = MapBuilder.create(() -> new EnumMap<TrackState, Track>(TrackState.class))
                .add(TrackState.REST, REST)
                .add(TrackState.FAR_ONE, FAR_1)
                .add(TrackState.FAR_TWO, FAR_2)
                .add(TrackState.FAR_THREE, FAR_3)
                .add(TrackState.FAR_FOUR, FAR_4)
                .add(TrackState.CLOSE_ONE, CLOSE_1)
                .add(TrackState.CLOSE_TWO, CLOSE_2)
                .add(TrackState.CLOSE_THREE, CLOSE_3)
                .add(TrackState.CLOSE_FOUR, CLOSE_4)
                .add(TrackState.FAR_AUTO, FAR_AUTO)
                .getMap();
    }

    public static Track trackCalibration(double hoodPos, double flywheelVel, double turretPos, AllianceColor allianceColor) {
        return new Track(hoodPos, flywheelVel,
                allianceColor.equals(AllianceColor.Blue) ? ServoTurret.turretPos.apply(turretPos) : turretPos,
                allianceColor.equals(AllianceColor.Blue) ? turretPos : ServoTurret.turretPosInv.apply(turretPos));
    }

    public static Track trackCalibration(double hoodPos, double flywheelVel, double turretPosRed, double turretPosBlue) {
        return new Track(hoodPos, flywheelVel, turretPosRed, turretPosBlue);
    }

    public static Track trackCalibration(double turretPosRed, double turretPosBlue) {
        return new Track(0, 0, turretPosRed, turretPosBlue);
    }

    public static Track trackCalibration(double turretPos, AllianceColor allianceColor) {
        return new Track(0, 0,
                allianceColor.equals(AllianceColor.Blue) ? ServoTurret.turretPos.apply(turretPos) : turretPos,
                allianceColor.equals(AllianceColor.Blue) ? turretPos : ServoTurret.turretPosInv.apply(turretPos));
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

        double omegaComp = pinpoint.getPinpoint().getHeadingVelocity(UnnormalizedAngleUnit.RADIANS) * ANGULAR_CONSTANT;
        double pos = ServoTurret.radToTicks(
                MathUtil.normalizeAnglePi(
                ServoTurret.ticksToRad(target.turretPos() + offsetInterpol.interpolate(pinpointPose.getHeading())) +
                        pinpointPose.getHeading() + omegaComp
                )
        );
        pos -= GyroThread.NEUTRAL_OFFSET * Math.signum(ServoTurret.REST - pos);
        return pos;
    }

    public BiConsumer<Flywheel, Hood> setState(TrackState state, boolean trackTraj) {
        return (f, h) -> {
            this.target = heatMap.get(state);
            if (target == null) throw new IllegalArgumentException();

            if (state.isFar() && trackTraj) {
                f.setVelocity(target.flywheelVel());
                h.setPosition(target.hoodPos());
            }
        };
    }
}
