package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
public class Tracking {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private OpenCvWebcam camera;
    public SkystoneDetector detector;
    public Tracking(HardwareMap hwmap, Telemetry telemetry){
        hardwareMap=hwmap;
        this.telemetry=telemetry;
    }
    public void initializeCamera(){
        detector=new SkystoneDetector();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera=new OpenCvWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"),cameraMonitorViewId);
        camera.openCameraDevice();
        camera.setPipeline(detector);
        camera.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);
    }
    private double getSkystoneX(){
        return detector.getScreenPosition().x;
    }

    private double getSkyStoneY() {
        return detector.getScreenPosition().y;
    }
}

