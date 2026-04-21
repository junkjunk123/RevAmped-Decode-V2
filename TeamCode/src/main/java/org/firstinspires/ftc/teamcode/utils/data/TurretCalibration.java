package org.firstinspires.ftc.teamcode.utils.data;

import org.firstinspires.ftc.teamcode.mechanisms.shooter.ServoTurret;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;

public class TurretCalibration {
    public final double posRed;
    public final double posBlue;

    public TurretCalibration(double positionRed, double positionBlue) {
        posRed = positionRed;
        posBlue = positionBlue;
    }

    public static TurretCalibration fromRed(double posRed) {
        return new TurretCalibration(posRed, ServoTurret.turretPosInv.apply(posRed));
    }

    public static TurretCalibration fromBlue(double posBlue) {
        return new TurretCalibration(ServoTurret.turretPos.apply(posBlue), posBlue);
    }

    public double getPos() {
        if (Globals.allianceColor.equals(AllianceColor.Red)) return posRed;
        return posBlue;
    }

    public TurretCalibration withBlue(double posBlue) {
        return new TurretCalibration(posRed, posBlue);
    }

    public TurretCalibration withRed(double posRed) {
        return new TurretCalibration(posRed, posBlue);
    }
}
