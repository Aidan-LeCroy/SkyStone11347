package org.firstinspires.ftc.teamcode.TestDebugOpModes;

import android.graphics.BitmapFactory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CircuitRunners.CheemsCVPipeline;
import org.firstinspires.ftc.teamcode.R;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Locale;

@TeleOp
public class VisionStream extends LinearOpMode {


    private OpenCvCamera webcam;



    @Override
    public void runOpMode() throws InterruptedException {
        CheemsCVPipeline pipeline = new CheemsCVPipeline();

        pipeline.initBit = BitmapFactory.decodeResource(hardwareMap.appContext.getResources(), R.drawable.cheems);
        pipeline.showPic = true;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        webcam.openCameraDevice();

        webcam.setPipeline(pipeline);


        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        pipeline.showPic = false;

        while (opModeIsActive()){

            telemetry.addData("Skystone Position", pipeline.skystone);

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
        }


        webcam.stopStreaming();
        webcam.closeCameraDevice();






    }


}
