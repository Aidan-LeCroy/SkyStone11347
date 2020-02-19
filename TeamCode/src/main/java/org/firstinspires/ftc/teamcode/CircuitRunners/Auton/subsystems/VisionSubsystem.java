package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CircuitRunners.CheemsCVPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Locale;

public class VisionSubsystem implements Subsystem {

    //Dimensions of the stream
    private static final int HEIGHT = 480;
    private static final int WIDTH = 640;

    //The orientation of the webcam
    private static final OpenCvCameraRotation webcamRotation = OpenCvCameraRotation.UPSIDE_DOWN;

    //The skystone position
    private static int skystonePos = -1;

    //The pipeline
    private CheemsCVPipeline pipeline;

    //The webcam
    private OpenCvCamera webcam;

    private LinearOpMode opMode;

    public VisionSubsystem(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public void initialize() {
        pipeline = new CheemsCVPipeline();

        //This is the view ID for the viewport
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                opMode.hardwareMap.appContext.getPackageName()
        );

        //Builds the camera
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                opMode.hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId
        );
    }

    public void startStreaming(){
        webcam.openCameraDevice();

        webcam.setPipeline(pipeline);

        //Start!
        webcam.startStreaming(WIDTH, HEIGHT, webcamRotation);
    }


    //Adds telemetry (doesn't update)
    public void addTelemetry(){
        skystonePos = pipeline.skystone;

        opMode.telemetry.addData("Skystone Position", pipeline.skystone);

        opMode.telemetry.addData("Frame Count", webcam.getFrameCount());
        opMode.telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
        opMode.telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        opMode.telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        opMode.telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        opMode.telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
    }

    //Returns the last known skystone position
    public int getSkystonePos(){
        skystonePos = pipeline.skystone;
        return skystonePos;
    }

    //Basically pauses the stream
    public void disable(){
        webcam.stopStreaming();
    }

    //Stops the stream
    public void stopStreaming(){
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    @Override
    public void stop(){
        //stopStreaming() is called wayyyy before stop(), so nothing happens here
    }
}
