package org.firstinspires.ftc.teamcode.utils.math.projectile;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;

public enum TrackState {
    REST,
    FAR_ONE,
    FAR_TWO,
    FAR_THREE,
    FAR_FOUR,
    CLOSE_ONE,
    CLOSE_TWO,
    CLOSE_THREE,
    CLOSE_FOUR,
    FAR_AUTO;

    public boolean isFar() {
        return this.equals(FAR_ONE) || this.equals(FAR_TWO) || this.equals(FAR_THREE) || this.equals(FAR_FOUR) || this.equals(FAR_AUTO);
    }

    public record Track(double hoodPos, double flywheelVel, double turretPos) {
        double getTurretPosFromRedCalibration() {
            return ServoTurret.turretPosInv.apply(turretPos);
        }

        double getTurretPosFromBlueCalibration() {
            return ServoTurret.turretPos.apply(turretPos);
        }
    }
}
