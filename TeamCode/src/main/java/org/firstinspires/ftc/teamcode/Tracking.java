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
    private String pipeline;
    public static final double DEGREES_PER_PIXEl=60/Math.sqrt(640^2+480^2);
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

        camera.startStreaming(640,480,OpenCvCameraRotation.UPRIGHT);
    }
    public double getSkyStoneX(){
        return detector.getScreenPosition().x;
    }

    public double getSkyStoneY() {
        return detector.getScreenPosition().y;
    }
//    Computing the angle requires only simple linear interpolation. For example, let's assume a camera with a resolution of 1920x1080 that covers a 45 degree angle of view across the diagonal.
//
//In this case, sqrt(1920^2 + 1080^2) gives 2292.19 pixels along the diagonal. That means each pixel represents 45/2292.19 = .0153994 degrees.
    // Explanation from StackOverflow user Jerry Coffin
    public double getAngletoSkyStone(){
        double skyX= getSkyStoneX();
        if (getSkyStoneX()<240)
            skyX-=480;
        return skyX*DEGREES_PER_PIXEl;
    }

}

