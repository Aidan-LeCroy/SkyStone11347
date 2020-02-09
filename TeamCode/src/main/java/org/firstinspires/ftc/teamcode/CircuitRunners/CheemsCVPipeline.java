package org.firstinspires.ftc.teamcode.CircuitRunners;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class CheemsCVPipeline extends OpenCvPipeline {



    //Example of Mat (image) object. This will hold a YCbCr frame
    private Mat yCbCrMat = new Mat();

    //This will hold the Cb channel (like a negative) of the YCbCr image
    private Mat cBmat = new Mat();

    //Always good to define the dimensions of the webcam
    private final double width = 640;
    private final double height = 480;

    // Width and height of the sampling rectangles
    private final double samplingRectWidth = 15;
    private final double samplingRectHeight = 15;



    //    private val samplePointPercentages = arrayOf(
    //        arrayOf(.15, .5), arrayOf(.5, .5), arrayOf(.85, .5)
    //    )

    private List<Mat> sectionRects(List<Point[]> samplePoints, Mat input) {
        List<Mat> samples = new ArrayList<>();
        samplePoints.forEach(
                (Point[] points) -> {
                    samples.add(
                            input.submat(new Rect(
                                            points[0],
                                            points[1]
                                    )
                            )
                    );
                }
        );
        return samples;

    }
    private List<Point[]> makePoints( List<Double[]> spots){
        List<Double[]> sampleSpots = new ArrayList<>(spots);
        List<Point[]> samplePoints = new ArrayList<>();
        sampleSpots.forEach(
                (Double[] points) -> {
                    samplePoints.add(
                            new Point[] {
                                    new Point(
                                            points[0] * width - samplingRectWidth / 2,
                                            points[1] * height - samplingRectHeight / 2
                                    ),
                                    new Point(
                                            points[0] * width + samplingRectWidth / 2,
                                            points[1] * width +samplingRectHeight / 2
                                    )

                            }
                    );
                }
        );
        return samplePoints;
    }


    private List<Double[]> sampleSpots = Arrays.asList(

     new Double[][] {{.3,.5},{.5,.5},{.7,.5}}
    );




    //This is the process the frames from the camera go through (the pipeline)
    //The input is the raw frame
    //The returned frame goes to the RC viewport
    //You can store data from this pipeline; ie. The skystone location
    @Override
    public Mat processFrame(Mat input){



        //First thing to do (for vision at least) is to convert the raw frame to a different color sspace
        //The input is in RGB, but RGB sucks. We will use YCbCr.
        Imgproc.cvtColor(input, yCbCrMat, Imgproc.COLOR_RGB2YCrCb); //Copies the input to another Mat

        //Next, we need to get the Cb aspect of the image, which shows the presence of
        //blueish color, but mostly dark color
        //No, I don't know why it's 2, but it works by golly
        Core.extractChannel(yCbCrMat, cBmat, 2);

        //You now have the Cb channel!
        //This shows darker ares, so next you should sample certain areas
        //And average out their average pixels
        //Then find the maximum value of all the areas (ie. max)

        ///For more reference, I strongly suggest looking at Noah's pipeline
        //It's very helpful, although it is in Kotlin
        //Some Kotlin things don't exist in Java, but the intention is the same, and
        //you can certainly find some way to do it
        //link: https://github.com/ftc16626/Skystone-2019/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/NaiveRectangleSamplingSkystoneDetectionPipeline.kt



        //Things:
        //Scalar represent colors in RGB format
        Scalar CircuitRunnersGreen = new Scalar(0.0, 255.0, 0.0); //R: 0, G: 255, B: 0

        //Point is exactly what it sounds like. A point.
        //NOTE: x and y in OpenCv is from the top left corner of an image, and the unit are pixels
        //Because each webcam res can be different, it's usually safer to use a relative value with
        //respect towards the image it is meant for
        Point myPoint = new Point(0.5 * width, 0.5 * height); //ie. 50% across, 50% down
        Point myPoint2 = new Point(0.75 * width, 0.75 * height); //ie. 75% across, 75% down

        //Rect are rectangles
        //They aren't Mats really, but a generic shape
        //Formed by two Points which represent the locations of two opposite corners of the the Rect
        //Also formed by a single point and a width/length
        //They are used when drawing on the frame and to indicate where to sub-mat an image for sampling
        Rect myRect = new Rect(myPoint, myPoint2);

        //Finally Mat
        //Mats are simple put images
        //They can have things drawn on them, and can be sub-matted to get certain sections for sampling
        //They MUST be released if they are created in the pipeline (see below)
        Mat myMat = new Mat();
        Mat myMat2 = new Mat();
        //Have fun!
        List<Mat> moreMats = new ArrayList<>();
        moreMats.add(myMat);
        moreMats.add(myMat2);

        //There are many other things as well!
        //Like drawing a rectange
        Imgproc.rectangle(
                input,
                new Point(
                        input.cols()/4,
                        input.rows()/4),
                new Point(
                        input.cols()*(.75),
                        input.rows()*(.75)),
                new Scalar(0, 255, 0),
                4
        );

        //Or adding text
        Imgproc.putText(
                input,
                "CR Black CV",
                new Point(
                        input.cols()*(1f/4f),
                        input.rows()*(1f/4f)
                ),
                Imgproc.FONT_HERSHEY_PLAIN,
                2,
                new Scalar(0, 50, 255),
                2

        );


        //There is one more important thing to know.
        //To avoid memory leaks and to free up resources, you must release any
        //Mat objects that you create in the pipeline method (excluding those in the class, like
        //the yCbCr or the cBmat object) at the end

        //You can do this directly
        //myMat.release();
        //myMat2.release();

        //Or any way that works
        for(Mat m : moreMats){
            m.release();
        }


        //Return the frame to show on the viewport. Use the original (if altered) frame with the same dimensions
        return input;

    }





}