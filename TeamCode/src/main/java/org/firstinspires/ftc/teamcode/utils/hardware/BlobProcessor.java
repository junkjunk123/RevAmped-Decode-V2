package org.firstinspires.ftc.teamcode.utils.hardware;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

/**
 * Blob Processor for Cameras
 * Set either boxFitColor or circleFitColor to 0 to disable them
 * @author Eric Zhao
 * @see <a href="https://ftc-docs.firstinspires.org/en/latest/color_processing/color-blob-concepts/color-blob-concepts.html">Tutorial</a>
 * @see org.firstinspires.ftc.robotcontroller.external.samples.ConceptVisionColorLocator_Circle
 * @see org.firstinspires.ftc.robotcontroller.external.samples.ConceptVisionColorLocator_Rectangle
 */
@Config
public class BlobProcessor {
    public ColorBlobLocatorProcessor blobProcessor;
    public List<ColorBlobLocatorProcessor.Blob> blobs;
    public static boolean debug = true;
    public static int blur = 15;
    public static int erode = 0;
    public static int dilate = 0;
    public static int boxFitColor = Color.rgb(255,120,31);
    public static int circleFitColor = 0;
    public static int roiColor = Color.rgb(255,255,255);
    public static int contourColor = Color.rgb(3,227,252);
    public static ColorBlobLocatorProcessor.MorphOperationType morphOperation = ColorBlobLocatorProcessor.MorphOperationType.CLOSING;

    /**
     * Blob detection using Camera
     * @param region region in the camera to detect
     * @param targetRange range of hues to detect color
     */
    public BlobProcessor(ImageRegion region, ColorRange targetRange) {
        blobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(targetRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(region)
                .setDrawContours(debug)
                .setBoxFitColor(boxFitColor)
                .setCircleFitColor(circleFitColor)
                .setRoiColor(roiColor)
                .setContourColor(contourColor)
                .setBlurSize(blur)
                .setErodeSize(erode)
                .setDilateSize(dilate)
                .setMorphOperationType(morphOperation)
                .build();
    }

    /**
     * Blob detection using camera's entire frame
     * @param targetRange range of hues to detect color
     */
    public BlobProcessor(ColorRange targetRange){
        this(ImageRegion.entireFrame(),targetRange);
    }

    /**
     * Blob detection using a sub-region of the camera
     * Region is defined by normal coordinates from 0 - height/width respectively
     * @param left the left X coordinate of the sub-region
     * @param top the top Y coordinate of the sub-region
     * @param right the right X coordinate of the sub-region
     * @param bottom the bottom Y coordinate of the sub-region
     * @param targetRange range of hues to detect color
     */
    public BlobProcessor(int left, int top, int right, int bottom, ColorRange targetRange){
        this(ImageRegion.asImageCoordinates(left,top,right,bottom),targetRange);
    }

    /**
     * Blob detection using a sub-region of the camera
     * Region is defined by a normalized coordinate grid from -1.0 - 1.0 for both x and y axis
     * @param left the left X coordinate of the sub-region
     * @param top the top Y coordinate of the sub-region
     * @param right the right X coordinate of the sub-region
     * @param bottom the bottom Y coordinate of the sub-region
     * @param targetRange range of hues to detect color
     */
    public BlobProcessor(double left, double top, double right, double bottom, ColorRange targetRange){
        this(ImageRegion.asUnityCenterCoordinates(left,top,right,bottom),targetRange);
    }

    public void update(){
        blobs = blobProcessor.getBlobs();
    }

    /**
     * Filters the blobs based on a criteria
     * @param criteria criteria you want to filter by
     * @param minValue min value for the criteria
     * @param maxValue max value for the criteria
     */
    public void filterBlobs (ColorBlobLocatorProcessor.BlobCriteria criteria, int minValue, int maxValue){
        ColorBlobLocatorProcessor.Util.filterByCriteria(criteria,minValue,maxValue,blobs);
    }

    /**
     * Sorts the blob list based on a criteria
     * @param criteria criteria you want to sort by
     * @param order sort by ascending or descending
     */
    public void sortBlobs (ColorBlobLocatorProcessor.BlobCriteria criteria, SortOrder order){
        ColorBlobLocatorProcessor.Util.sortByCriteria(criteria, order, blobs);
    }

    /**
     * Gets the blobs detected
     * @return the blobs that are detected
     */
    public List<ColorBlobLocatorProcessor.Blob> getBlobs(){
        return blobs;
    }

    /**
     * Adds a prefilter to the camera
     * @param filter the prefilter you want to add
     */
    public void addPreFilter(ColorBlobLocatorProcessor.BlobFilter filter){
        blobProcessor.addFilter(filter);
    }

    /**
     * Removes a prefilter from the camera
     * @param filter the prefilter you want to remove
     */
    public void removePreFilter(ColorBlobLocatorProcessor.BlobFilter filter){
        blobProcessor.removeFilter(filter);
    }

    /**
     * Removes all prefilters from the camera
     */
    public void removeAllPreFilters(){
        blobProcessor.removeAllFilters();
    }

    /**
     * Sets a presort for the blobs
     * @param sort the sort you want to add
     */
    public void setPreSort(ColorBlobLocatorProcessor.BlobSort sort){
        blobProcessor.setSort(sort);
    }

    public ColorBlobLocatorProcessor getProcessor(){
        return blobProcessor;
    }

}
