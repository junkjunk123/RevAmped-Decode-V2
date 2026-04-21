package org.firstinspires.ftc.teamcode.utils.math.projectile;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;

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
        return equals(FAR_ONE) || equals(FAR_TWO) || equals(FAR_THREE) || equals(FAR_FOUR) || equals(FAR_AUTO);
    }

    public record Track(double hoodPos, double flywheelVel, double turretPosRed, double turretPosBlue) {
        public double turretPos() {
            if (Globals.allianceColor.equals(AllianceColor.Red)) return turretPosRed;
            return turretPosBlue;
        }
    }
}
