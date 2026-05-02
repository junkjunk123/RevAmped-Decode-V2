package org.firstinspires.ftc.teamcode.utils.math.projectile;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.GyroThread;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.data.ListMap;
import org.firstinspires.ftc.teamcode.utils.data.MapBuilder;
import org.firstinspires.ftc.teamcode.utils.math.ILUT;
import org.firstinspires.ftc.teamcode.utils.math.MathUtil;
import org.firstinspires.ftc.teamcode.utils.math.projectile.TrackState.Track;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;
import java.util.function.BiConsumer;

import smile.interpolation.LinearInterpolation;

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

    //public static ILUT offsetInterpol;
    public static ILUT offsetInterpol;

    private Track target;
    private boolean reversed = false;

    public static double ANGULAR_CONSTANT = 0.06;
    public static double[] offsets;

    public FarTrackingMath(Localizer pinpoint) {
        this.pinpoint = (PinpointLocalizer) pinpoint;

        REST = trackCalibration(ServoTurret.REST, AllianceColor.Blue);

        FAR_1 = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY - 15,
                212/255d, 185/255d);
        FAR_2 = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY - 5,
                218/255d, 181/255d);
        FAR_3 = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY + 15,
                221/255d, 174/255d);
        FAR_4 = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY + 45,
                225/255d, 175/255d);

        FAR_AUTO = trackCalibration(Hood.FAR_PRESET, Flywheel.FAR_VELOCITY - 45,
                214/255d, 180/255d);

        CLOSE_1 = trackCalibration(231/255d, 162/255d);
        CLOSE_2 = trackCalibration(250/255d, 149/255d);
        CLOSE_3 = trackCalibration(-26/255d, 139/255d);
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
                .build();
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
        double offset;

        if (Globals.isTeleOp) {
            if (reversed) {
                if (Globals.allianceColor.equals(AllianceColor.Red)) offset = offsets[5];
                else offset = 0;
            } else {
                if (Globals.allianceColor.equals(AllianceColor.Red)) offset = 0;
                else offset = offsets[5];
            }
        } else {
            offset = offsetInterpol.interpolate(MathFunctions.normalizeAngle(pinpointPose.getHeading()));
        }

        double pos = ServoTurret.radToTicks(
                MathUtil.normalizeAnglePi(
                ServoTurret.ticksToRad(target.turretPos() + offset //offsetInterpol.interpolate(MathFunctions.normalizeAngle(pinpointPose.getHeading()))
                ) + pinpointPose.getHeading() + omegaComp
                )
                //indentation is so interpolation can be commented out easily
        );
        pos -= GyroThread.NEUTRAL_OFFSET * Math.signum(ServoTurret.REST - pos);
        pos = Range.clip(pos, 0.0, 1.0);
        return pos;
    }

    public BiConsumer<Flywheel, Hood> setState(TrackState state, boolean trackTraj) {
        return (f, h) -> {
            this.target = heatMap.get(state);
            if (target == null) throw new IllegalArgumentException();
            if (state.equals(TrackState.FAR_THREE) || state.equals(TrackState.FAR_FOUR)) reversed = true;
            else reversed = false;

            if (state.isFar() && trackTraj) {
                f.setVelocity(target.flywheelVel());
                //h.setPosition(target.hoodPos());
            }
        };
    }

    public static void buildOffsetILUT(ListMap<Double, Double> turretOffsets) {
        Double[] x = turretOffsets.getMap().keySet().toArray(new Double[0]);
        Double[] y = turretOffsets.getMap().values().toArray(new Double[0]);

        double[] xVals = new double[x.length];
        double[] yVals = new double[y.length];

        for (int i = 0; i < x.length; i++) {
            double heading = x[i];
            xVals[i] = heading;
            yVals[i] = ServoTurret.radToTicks(ServoTurret.ticksToRad(y[i]) - heading) - y[0];
            offsets = yVals;
        }

        //offsetInterpol = new LinearInterpolation(xVals, yVals);

        ILUT.Builder builder = new ILUT.Builder();
        double initial = turretOffsets.get(0).val();
        for (int i = 0; i < turretOffsets.size(); i++) {
            ListMap.Entry<Double, Double> calibration = turretOffsets.get(i);
            double heading = calibration.key();
            builder.add(heading, ServoTurret.radToTicks(ServoTurret.ticksToRad(calibration.val()) - heading) - initial);
        }
        offsetInterpol = builder.build();
    }
}
