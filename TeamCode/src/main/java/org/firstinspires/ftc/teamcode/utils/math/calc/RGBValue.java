package org.firstinspires.ftc.teamcode.utils.math.calc;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

/**
 * A class representing an RGB color value.
 * It provides methods to set and get RGB values, check color dominance,
 * and convert RGB to HSV (Hue, Saturation, Value).
 */
public class RGBValue {
    private double R=0,G=0,B=0;

    public RGBValue(NormalizedRGBA rgba) {
        this.R = rgba.red * 255.0;
        this.G = rgba.green * 255.0;
        this.B = rgba.blue * 255.0;
    }

    public RGBValue (int R, int G, int B) {
        this.R=R;
        this.G=G;
        this.B=B;
    }

    public RGBValue (double R, double G, double B) {
        this.R=R;
        this.G=G;
        this.B=B;
    }

    public void setRGB(int r, int g, int b) {
        R = r;
        G = g;
        B = b;
    }

    public void setRGB(double r, double g, double b) {
        R = r;
        G = g;
        B = b;
    }

    public void setB(int b) {
        B = b;
    }

    public void setG(int g) {
        G = g;
    }

    public void setR(int r) {
        R = r;
    }

    public int getIntegerR() {return (int) Math.round(R);}

    public int getIntegerG() {return (int) Math.round(G);}

    public int getIntegerB() {return (int) Math.round(B);}

    public double getB() {
        return B;
    }

    public double getG() {
        return G;
    }

    public double getR() {
        return R;
    }

    public boolean isRed() {
        return (R > G && R>B);
    }

    public boolean isBlue() {
        return (B > R && B>G);
    }

    public boolean isGreen() {
        return (G > R && G>B);
    }

    /**
     * Converts RGB to HSV
     * @param r the red value
     * @param g the green value
     * @param b the blue value
     * @return an HSVValue object containing the hue, saturation, and value
     */
    public static HSVValue rgb_to_hsv(double r, double g, double b) {

        // R, G, B values are divided by 255
        // to change the range from 0..255 to 0..1
        r = r / 255.0;
        g = g / 255.0;
        b = b / 255.0;

        // h, s, v = hue, saturation, value
        double cmax = Math.max(r, Math.max(g, b)); // maximum of r, g, b
        double cmin = Math.min(r, Math.min(g, b)); // minimum of r, g, b
        double diff = cmax - cmin; // diff of cmax and cmin.
        double h = -1, s = -1;

        // if cmax and cmax are equal then h = 0
        if (cmax == cmin)
            h = 0;

            // if cmax equal r then compute h
        else if (cmax == r)
            h = (60 * ((g - b) / diff) + 360) % 360;

            // if cmax equal g then compute h
        else if (cmax == g)
            h = (60 * ((b - r) / diff) + 120) % 360;

            // if cmax equal b then compute h
        else if (cmax == b)
            h = (60 * ((r - g) / diff) + 240) % 360;

        // if cmax equal zero
        if (cmax == 0)
            s = 0;
        else
            s = (diff / cmax);

        // compute v
        double v = cmax;

        return new HSVValue(h, s, v);
    }

    public HSVValue toHSV() {
        return rgb_to_hsv(R, G, B);
    }

    @NonNull
    @Override
    public String toString() {
        return "R: " + R + " G: " + G + " B: " + B;
    }
}
