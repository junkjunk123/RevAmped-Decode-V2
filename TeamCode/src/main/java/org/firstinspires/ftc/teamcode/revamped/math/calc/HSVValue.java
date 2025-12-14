package org.firstinspires.ftc.teamcode.revamped.math.calc;

import androidx.annotation.NonNull;

/**
 * A class representing a color in the HSV (Hue, Saturation, Value) color space.
 * This class provides methods to set and get the HSV values, as well as calculate
 * the distance between two HSV colors.
 */
public class HSVValue {
    private double H;
    private double S;
    private double V;

    /**
     * Constructs an HSVValue with the specified hue, saturation, and value.
     *
     * @param H the hue of the color
     * @param S the saturation of the color
     * @param V the value (brightness) of the color
     */
    public HSVValue(double H, double S, double V) {
        this.H = H;
        this.S = S;
        this.V = V;
    }

    /**
     * Constructs an HSVValue with default values (0, 0, 0).
     */
    public void setHSV(double h, double s, double v) {
        H = h;
        S = s;
        V = v;
    }

    public void setH(double h) {
        H = h;
    }

    public void setS(double s) {
        S = s;
    }

    public void setV(double v) {
        V = v;
    }

    public double getH() {
        return H;
    }

    public double getS() {
        return S;
    }

    public double getV() {
        return V;
    }

    @NonNull
    @Override
    public String toString() {
        return "H: " + H + " S: " + S + " V: " + V;
    }

    /**
     * Calculates the distance between two HSV colors using a specific formula.
     * The formula is based on the differences in hue, saturation, and value.
     *
     * @param h1 the hue of the first color
     * @param s1 the saturation of the first color
     * @param v1 the value of the first color
     * @param h2 the hue of the second color
     * @param s2 the saturation of the second color
     * @param v2 the value of the second color
     * @return the calculated distance between the two colors
     */
    public static float calculateDistance(float h1, float s1, float v1, float h2, float s2, float v2) {
        return (h1 - h2) * (h1 - h2) / 1296 + (s1 - s2) * (s1 - s2) + (v1 - v2) * (v1 - v2);
    }

    /**
     * Finds the distance between two hues.
     * This method calculates the absolute difference between two hue values.
     *
     * @param h1 the first hue value
     * @param h2 the second hue value
     * @return the absolute distance between the two hues
     */
    public static float findDistanceHue(float h1, float h2) {
        return Math.abs(h1-h2);
    }
}
