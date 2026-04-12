package org.firstinspires.ftc.teamcode.utils.math.projectile;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;

public enum TrackState {
    FAR_ONE,
    FAR_TWO,
    FAR_THREE,
    CLOSE_ONE,
    CLOSE_TWO,
    CLOSE_THREE,
    CLOSE_FOUR;

    public boolean isFar() {
        return this.equals(FAR_ONE) || this.equals(FAR_TWO) || this.equals(FAR_THREE);
    }

    public record Track(double hoodPos, double flywheelVel, double turretPos) {
        double getTurretPosFromRedCalibration() {
            return ServoTurret.turretPosInv.apply(turretPos);
        }

        double getTurretPosFromBlueCalibration() {
            return ServoTurret.turretPosInv.apply(turretPos);
        }
    }
}
