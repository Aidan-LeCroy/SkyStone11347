package org.firstinspires.ftc.teamcode.Philobots;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Tracking;

@Autonomous(name="WA Autonomous 1.0", group="Diffy")
@Disabled
public class Auto extends OpMode {
    public enum SSPos{
        LEFT,
        CENTER,
        RIGHT
    }
    private Robot robot;
    private DriveController drive=robot.driveController;
    private Tracking track;
    private SSPos pos;
    @Override
    public void init() {
        track=new Tracking(hardwareMap,telemetry);
        track.initializeCamera();
    }
    public void init_loop(){
        double x=track.getSkyStoneX();
        double y=track.getSkyStoneY();
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