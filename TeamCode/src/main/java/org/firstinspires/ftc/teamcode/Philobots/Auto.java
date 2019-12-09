package org.firstinspires.ftc.teamcode.Philobots;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DiffCore;
import org.firstinspires.ftc.teamcode.Tracking;

@Autonomous(name="WA Autonomous 1.0", group="Diffy")
@Disabled
public class Auto extends OpMode {
    public enum SSPos{
        LEFT,
        CENTER,
        RIGHT
    }
    Robot robot;
    DriveController drive=robot.driveController;
    Tracking track;
    SSPos pos;
    SkystoneDetector detector;
    @Override
    public void init() {
        track=new Tracking(hardwareMap,telemetry);
        track.initializeCamera();
        detector=track.detector;
    }
    public void init_loop(){
        double x=detector.getScreenPosition().x;
        double y=detector.getScreenPosition().y;
        pos=SSPos.LEFT;
        pos=SSPos.RIGHT;
        pos=SSPos.CENTER;
    }

    @Override
    public void start() {
        robot.initIMU();
        super.start();
    }

    @Override
    public void loop() {
        drive.drive(Vector2d.FORWARD,40,.5,this,2000);


    }

    @Override
    public void stop() {
        super.stop();
    }
}