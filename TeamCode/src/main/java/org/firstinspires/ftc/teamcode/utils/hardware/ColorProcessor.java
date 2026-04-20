package org.firstinspires.ftc.teamcode.utils.hardware;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVisionColorSensor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

/**
 * ColorProcessor
 * @author Eric Zhao
 * @see <a href="https://ftc-docs.firstinspires.org/en/latest/color_processing/index.html">Tutorial</a>
 * @see ConceptVisionColorSensor
 */
public class ColorProcessor {
    public PredominantColorProcessor colorProcessor;
    public PredominantColorProcessor.Result result;

    /**
     * Custom color sensor using a camera
     * @param region region in the camera to detect
     * @param targetColors colors for the camera to detect
     */
    public ColorProcessor(ImageRegion region, PredominantColorProcessor.Swatch... targetColors){
        colorProcessor = new PredominantColorProcessor.Builder()
                .setRoi(region)
                .setSwatches(targetColors)
                .build();
    }

    /**
     * Custom color sensor using a camera's entire frame
     * @param targetColors colors the camera should detect
     */
    public ColorProcessor(PredominantColorProcessor.Swatch... targetColors){
        this(ImageRegion.entireFrame(),targetColors);
    }

    /**
     * Custom color sensor using a camera's sub-region
     * Region is defined by normal coordinates from 0 - height/width respectively
     * @param left the left X coordinate of the sub-region
     * @param top the top Y coordinate of the sub-region
     * @param right the right X coordinate of the sub-region
     * @param bottom the bottom Y coordinate of the sub-region
     * @param targetColors colors the camera should detect
     */
    public ColorProcessor(int left, int top, int right, int bottom, PredominantColorProcessor.Swatch... targetColors){
        this(ImageRegion.asImageCoordinates(left,top,right,bottom), targetColors);
    }

    /**
     * Custom color sensor using a camera's sub-region
     * Region is defined by a normalized coordinate grid from -1.0 - 1.0 for both x and y axis
     * @param left the left X coordinate of the sub-region
     * @param top the top Y coordinate of the sub-region
     * @param right the right X coordinate of the sub-region
     * @param bottom the bottom Y coordinate of the sub-region
     * @param targetColors colors the camera should detect
     */
    public ColorProcessor(double left, double top, double right, double bottom, PredominantColorProcessor.Swatch... targetColors){
        this(ImageRegion.asUnityCenterCoordinates(left,top,right,bottom), targetColors);
    }

    public void update(){
        result = colorProcessor.getAnalysis();
    }

    /**
     * Gets the color of what's detected
     * @return the closest color (Swatch object) that is detected
     */
    public PredominantColorProcessor.Swatch getColor(){
        return result.closestSwatch;
    }

    /**
     * Gets the RGB (Red, Green, Blue) values of what's detected
     * @return a 3-length int[] of the RGB values
     */
    public int[] getRGB(){
        return result.RGB;
    }

    /**
     * Gets the HSV (Hue, Saturation, Value) values of what's detected
     * @return a 3-length int[] of the HSV values
     */
    public int[] getHSV(){
        return result.HSV;
    }

    /**
     * Gets the YCrCb (Luma, Chroma Red, Chroma Blue) values of what's detected
     * @return a 3-length int[] of the YCrCb values
     */
    public int[] getYCrCb(){
        return result.YCrCb;
    }

    /**
     * Gets the color of what's detected
     * @return the String of the Swatch that is detected
     */
    public String getStringColor(){
        return result.closestSwatch.name();
    }

    /**
     *
     * @param targetColor the color (Swatch) you are checking
     * @return a boolean if the camera detects targetColor
     */
    public boolean detectsColor(PredominantColorProcessor.Swatch targetColor){
        return result.closestSwatch.equals(targetColor);
    }

    public PredominantColorProcessor getProcessor(){
        return colorProcessor;
    }
}
