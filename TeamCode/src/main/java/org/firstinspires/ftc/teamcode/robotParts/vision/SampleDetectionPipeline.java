package org.firstinspires.ftc.teamcode.robotParts.vision;

import com.acmerobotics.dashboard.config.Config;

import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

//Thanks to WutIsHummus / Alpeeen from #25679.
public class SampleDetectionPipeline extends OpenCvPipeline {
    boolean debug;
    public SampleDetectionPipeline(boolean localDebug){debug = localDebug;}
    /*
     * Working image buffers
     */
    Mat YCbCrMat = new Mat();
    Mat CrMat = new Mat();
    Mat CbMat = new Mat();

    Mat blueThresholdMat = new Mat();
    Mat redThresholdMat = new Mat();
    Mat yellowThresholdMat = new Mat();

    Mat morphedBlueThreshold = new Mat();
    Mat morphedRedThreshold = new Mat();
    Mat morphedYellowThreshold = new Mat();

    Mat contoursOnPlainImageMat = new Mat();

    /*
     * Threshold values
     */
    public static int
            AREA_LOWER_LIMIT = 3000,
            AREA_UPPER_LIMIT = 20000,
            YELLOW_MASK_THRESHOLD = 77,
            BLUE_MASK_THRESHOLD = 150,
            RED_MASK_THRESHOLD = 170;

    /*
     * Elements for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));

    /*
     * Colors
     */
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar YELLOW = new Scalar(255, 255, 0);

    static final int CONTOUR_LINE_THICKNESS = 2;

    public static class Sample {
        double angle;
        Size size;
        Point position;
        String color;
    }

    ArrayList<Sample> internalSampleList = new ArrayList<>();
    volatile ArrayList<Sample> clientSampleList = new ArrayList<>();

    /*
     * Viewport stages
     */
    enum Stage {
        FINAL,
        YCrCb,
        MASKS,
        MASKS_NR,
        CONTOURS
    }

    Stage[] stages = Stage.values();
    int stageNum = 0;


    @Override
    public void onViewportTapped() {
        int nextStageNum = stageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }

    @Override
    public Mat processFrame(Mat input) {
        internalSampleList.clear();

        /*
         * Run the image processing
         */
        findContours(input);

        clientSampleList = new ArrayList<>(internalSampleList);

        /*
         * Decide which buffer to send to the viewport
         */
        switch (stages[stageNum]) {
            case YCrCb: {
                return YCbCrMat;
            }

            case MASKS: {
                Mat masks = new Mat();
                Core.addWeighted(yellowThresholdMat, 1.0, redThresholdMat, 1.0, 0.0, masks);
                Core.addWeighted(masks, 1.0, blueThresholdMat, 1.0, 0.0, masks);
                return masks;
            }

            case MASKS_NR: {
                Mat masksNR = new Mat();
                Core.addWeighted(morphedYellowThreshold, 1.0, morphedRedThreshold, 1.0, 0.0, masksNR);
                Core.addWeighted(masksNR, 1.0, morphedBlueThreshold, 1.0, 0.0, masksNR);
                return masksNR;
            }

            case CONTOURS: {
                return contoursOnPlainImageMat;
            }

            default: {
                return input;
            }
        }
    }

    public ArrayList<Sample> getDetectedStones() {
        return clientSampleList;
    }

    void findContours(Mat input) {
        // Convert the input image to YCrCb color space
        Imgproc.cvtColor(input, YCbCrMat, Imgproc.COLOR_RGB2YCrCb);

        // Extract the Cb and Cr channels
        Core.extractChannel(YCbCrMat, CbMat, 2); // Cb channel index is 2
        Core.extractChannel(YCbCrMat, CrMat, 1); // Cr channel index is 1

        // Threshold the channels to form masks
        Imgproc.threshold(CbMat, blueThresholdMat, BLUE_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(CrMat, redThresholdMat, RED_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(CbMat, yellowThresholdMat, YELLOW_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);

        // Apply morphology to the masks
        morphMask(blueThresholdMat, morphedBlueThreshold);
        morphMask(redThresholdMat, morphedRedThreshold);
        morphMask(yellowThresholdMat, morphedYellowThreshold);

        // Find contours in the masks
        ArrayList<MatOfPoint> blueContoursList = new ArrayList<>();
        Imgproc.findContours(morphedBlueThreshold, blueContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        ArrayList<MatOfPoint> redContoursList = new ArrayList<>();
        Imgproc.findContours(morphedRedThreshold, redContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        ArrayList<MatOfPoint> yellowContoursList = new ArrayList<>();
        Imgproc.findContours(morphedYellowThreshold, yellowContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Create a plain image for drawing contours
        contoursOnPlainImageMat = Mat.zeros(input.size(), input.type());

        // Analyze and draw contours
        for(MatOfPoint contour : blueContoursList) {
            analyzeContour(contour, input, "Blue");
        }

        for(MatOfPoint contour : redContoursList) {
            analyzeContour(contour, input, "Red");
        }

        for(MatOfPoint contour : yellowContoursList) {
            analyzeContour(contour, input, "Yellow");
        }
    }

    void morphMask(Mat input, Mat output) {
        /*
         * Apply erosion and dilation for noise reduction
         */
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void analyzeContour(MatOfPoint contour, Mat input, String color) {
        // Transform the contour to a different format
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);

        // Fit a rotated rectangle to the contour and draw it
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

        if(rotatedRectFitToContour.size.area() > AREA_LOWER_LIMIT && rotatedRectFitToContour.size.area() < AREA_UPPER_LIMIT) {

            // Adjust the angle based on rectangle dimensions
            double rotRectAngle = rotatedRectFitToContour.angle;
            if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height)
            {
                rotRectAngle += 90;
            }

            // Compute the angle and store it
            double angle = -(rotRectAngle - 180);

            // Store the detected stone information
            Sample sample = new Sample();
            sample.size = rotatedRectFitToContour.size;
            sample.position = rotatedRectFitToContour.center;
            sample.angle = rotRectAngle;
            sample.color = color;
            internalSampleList.add(sample);

            if(debug) {
                drawRotatedRect(rotatedRectFitToContour, input, color);
                drawRotatedRect(rotatedRectFitToContour, contoursOnPlainImageMat, color);
                drawTagText(rotatedRectFitToContour,
                        Integer.toString((int) Math.round(angle)) + " deg " +
                                Integer.toString((int) Math.round(rotatedRectFitToContour.size.area())) + " area " +
                                Integer.toString((int) Math.round(rotatedRectFitToContour.boundingRect().area())),
                        input, color);
            }
        }
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat, String color) {
        Scalar colorScalar = getColorScalar(color);

        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        rect.center.x - 50,  // x anchor point
                        rect.center.y + 25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1, // Font size
                colorScalar, // Font color
                1); // Font thickness
    }

    static void drawRotatedRect(RotatedRect rect, Mat drawOn, String color) {
        /*
         * Draws a rotated rectangle by drawing each of the 4 lines individually
         */
        Point[] points = new Point[4];
        rect.points(points);

        for (int i = 0; i < 4; ++i) {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], getColorScalar(color), 2);
            Imgproc.circle(drawOn,rect.center,15,getColorScalar(color));
        }
    }

    static Scalar getColorScalar(String color) {
        switch (color)
        {
            case "Blue":
                return BLUE;
            case "Yellow":
                return YELLOW;
            default:
                return RED;
        }
    }
}