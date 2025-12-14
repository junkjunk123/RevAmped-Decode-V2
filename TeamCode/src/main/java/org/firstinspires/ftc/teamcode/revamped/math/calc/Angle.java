package org.firstinspires.ftc.teamcode.revamped.math.calc;

import com.qualcomm.robotcore.util.Range;

public class Angle {
    /**
     * Normalizes an angle in radians to the range [-π, π).
     *
     * @param angleRadians The angle in radians to normalize.
     * @return The normalized angle in radians, in the range [-π, π).
     */
    public static double normalizeAnglePi(double angleRadians) {
        return Math.atan2(Math.sin(angleRadians), Math.cos(angleRadians));
    }

    public static float servoPosFromRad(double rad, double minRad, double maxRad, float minRadPos, float maxRadPos) {
        if (minRad < maxRad) {
            float pos = (float) Range.scale(rad, minRad, maxRad, minRadPos, maxRadPos);
            if (minRadPos < maxRadPos) return pos;
            else return (minRadPos - pos) + maxRadPos;
        }

        float pos = (float) Range.scale(rad, maxRad, minRad, maxRadPos, minRadPos);
        if (maxRadPos < minRadPos) return pos;
        return (maxRadPos - pos) + minRadPos;
    }
}
