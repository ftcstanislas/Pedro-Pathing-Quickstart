package org.firstinspires.ftc.teamcode.robotParts.vision;

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

    final double
            xPixels = 448,
            yPixels = 800,
            cameraXPos = 5.0, //In init, primary axis (x is forwards/backwards) offset versus middle of the intake
            cameraYPos = -2.0, //Offset in secondary axis versus middle of the intake
            cameraZPos = 28.4, //Height of the mount
            cameraAlpha = 42.47, //In degrees, will be converted to radians later, 0 means parallel to the floor.
            yDegreePerPixel = 25 * Math.sqrt(3025.0/821.0) / yPixels,//14:25 ratio on the camera * sqrt ( 55 degrees squared / (14^2 + 25^2) ) = horizontal FOV, divided by pixels to get degree per pixel TODO maybe regression better
            xDegreePerPixel = 14 * Math.sqrt(3025.0/821.0) / xPixels; //TODO maybe regression better

    /*
     * Working image buffers
     */
    Mat YCbCrMat = new Mat(),
        CrMat = new Mat(),
        CbMat = new Mat();

    Mat blueThresholdMat = new Mat(),
        redThresholdMat = new Mat(),
        yellowThresholdMat = new Mat();

    Mat morphedBlueThreshold = new Mat(),
        morphedRedThreshold = new Mat(),
        morphedYellowThreshold = new Mat();

    Mat contoursOnPlainImageMat = new Mat();

    RotatedRect rotatedRectFitToContour;

    /*
     * Threshold values
     */
    public static int
            AREA_LOWER_LIMIT = 5000,
            AREA_UPPER_LIMIT = 100000,
            YELLOW_MASK_THRESHOLD = 77,
            BLUE_MASK_THRESHOLD = 150,
            RED_MASK_THRESHOLD = 170;

    /*
     * Elements for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5)),
        dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));

    /*
     * Colors for drawing on the Driver Station.
     */
    static final Scalar
            RED = new Scalar(255, 0, 0),
            BLUE = new Scalar(0, 0, 255),
            YELLOW = new Scalar(255, 255, 0),
            WHITE = new Scalar(255,255,255);

    public class Sample {
        double grabAngle;
        Size size;
        Point cameraPosition;
        double cameraYAngle,cameraXAngle;
        double
            actualX,actualY,
            actualAngle,
            xFromBorder,yFromBorder;
        Scalar color;
        double score;

        /**
         * Is cameraY, which is robotX, and is relative to the intake.
         */
        void inferY() {
            cameraYAngle = (cameraPosition.y - 0.5 * yPixels) * yDegreePerPixel;
            actualY = cameraZPos * Math.tan(Math.toRadians(cameraAlpha - cameraYAngle));
            //TODO: correct for intake offset
        }

        /**
         * Is cameraX, which is robotY, and is relative to the intake.
         */
        void inferX() {
            cameraXAngle = (cameraPosition.x - 0.5 * xPixels) * xDegreePerPixel;
            actualX = (cameraZPos / Math.cos(Math.toRadians(cameraAlpha)))*Math.tan(Math.toRadians(cameraXAngle)) * scaleX(cameraYAngle);
            //TODO: correct for intake offset
        }

        /**
         * Angle the robot would have to turn to align the slides with the sample.
         */
        void inferAngle() {
            actualAngle = Math.tan(actualX/actualY);
        }

        /**
         *
         */
        void assignSamplePoints() {
            score -= (int) actualY;
            score -= (int) (2 * Math.abs(actualX));
        }

        /**
         *
         * @param y
         * @return
         */
        double scaleX(double y) {
            return 1;
//            return Math.cos(Math.toRadians(9*Math.sqrt(3025.0/377.0))) * y + 1; //TODO regression
        }
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
    public Sample bestSample;


    @Override
    public void onViewportTapped() {
        int nextStageNum = stageNum + 1;

        if(nextStageNum >= stages.length) {
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
        getBestSample(internalSampleList, RED);

        clientSampleList = new ArrayList<>(internalSampleList);


        /*
         * Decide which buffer to send to the viewport
         */
        switch (stages[stageNum]) {
            case YCrCb:
                return YCbCrMat;

            case MASKS:
                Mat masks = new Mat();
                Core.addWeighted(yellowThresholdMat, 1.0, redThresholdMat, 1.0, 0.0, masks);
                Core.addWeighted(masks, 1.0, blueThresholdMat, 1.0, 0.0, masks);
                return masks;

            case MASKS_NR:
                Mat masksNR = new Mat();
                Core.addWeighted(morphedYellowThreshold, 1.0, morphedRedThreshold, 1.0, 0.0, masksNR);
                Core.addWeighted(masksNR, 1.0, morphedBlueThreshold, 1.0, 0.0, masksNR);
                return masksNR;

            case CONTOURS:
                return contoursOnPlainImageMat;

            default:
                return input;
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
            analyzeContour(contour, input, BLUE);
        }

        for(MatOfPoint contour : redContoursList) {
            analyzeContour(contour, input, RED);
        }

        for(MatOfPoint contour : yellowContoursList) {
            analyzeContour(contour, input, YELLOW);
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

    void analyzeContour(MatOfPoint contour, Mat input, Scalar color) {
        // Transform the contour to a different format
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);

        // Fit a rotated rectangle to the contour and draw it
        rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

        if(rotatedRectFitToContour.size.area() > AREA_LOWER_LIMIT && rotatedRectFitToContour.size.area() < AREA_UPPER_LIMIT) {

            // Adjust the angle based on rectangle dimensions
            double rotRectAngle = rotatedRectFitToContour.angle;
            if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
                rotRectAngle += 90;
            }

            // Compute the angle and store it
            double angle = -(rotRectAngle - 180);

            // Store the detected stone information
            Sample sample = new Sample();
            sample.size = rotatedRectFitToContour.size;
            sample.cameraPosition = rotatedRectFitToContour.center;
            sample.grabAngle = angle;
            sample.color = color;
            internalSampleList.add(sample);

            if(debug) {
                sample.inferY();
                sample.inferX();
                drawRotatedRect(rotatedRectFitToContour, input, color);
                drawRotatedRect(rotatedRectFitToContour, contoursOnPlainImageMat, color);
                drawTagText(
                    rotatedRectFitToContour,
//                (int) Math.round(angle) + " deg " +
//                    (int) Math.round(rotatedRectFitToContour.size.area()) + " area " +
                    sample.actualX + " x " +
                    sample.actualY + " y",
//                    (int) Math.round(sample.cameraXAngle) + " x " +
//                    (long) sample.cameraYAngle + " y ",
                    input, BLUE);
            }
        }
    }

    /**
     *
     * @param sampleList
     * @param color
     * @return
     */
    public Sample getBestSample(ArrayList<SampleDetectionPipeline.Sample> sampleList, Scalar color) {
        bestSample = new Sample();
        bestSample.score = 0;
        for (Sample sample : sampleList) {
            //TODO: remove wrong colors
//            if (sample.color != color) {
//                sampleList.remove(sample);
//                break;
//            }

//        sample.yFromBorder;
//        sample.xFromBorder;
            //TODO: if too close to border remove from sampleList

            sample.inferY();
            sample.inferX();
            sample.inferAngle();
            sample.assignSamplePoints();
            if (sample.score > bestSample.score) bestSample = sample;
        }
        return bestSample;
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat, Scalar color) {

        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        rect.center.x - 50,  // x anchor point
                        rect.center.y + 25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                3, // Font size
                color, // Font color
                1); // Font thickness
    }

    static void drawRotatedRect(RotatedRect rect, Mat drawOn, Scalar color) {
        /*
         * Draws a rotated rectangle by drawing each of the 4 lines individually
         */
        Point[] points = new Point[4];
        rect.points(points);

        for (int i = 0; i < 4; ++i) {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], color, 2);
            Imgproc.circle(drawOn,rect.center,15, color);
        }
    }
}