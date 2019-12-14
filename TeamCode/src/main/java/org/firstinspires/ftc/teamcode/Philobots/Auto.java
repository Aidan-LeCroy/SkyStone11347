package org.firstinspires.ftc.teamcode.Philobots;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Tracking;

@Autonomous(name="WA Autonomous 1.0", group="Diffy")
@Disabled
public class Auto extends LinearOpMode {
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
    public void runOpMode(){
        track=new Tracking(hardwareMap,telemetry);
        track.initializeCamera();
        robot=new Robot(this,true);
        robot.initIMU();

        while(!this.opModeIsActive()) {
            double x = track.getSkyStoneX();
            double y = track.getSkyStoneY();
            pos = SSPos.LEFT;
            pos = SSPos.RIGHT;
            pos = SSPos.CENTER;
        }
        waitForStart();
        drive.drive(Vector2d.FORWARD,20,.8,this,2500);

    }

}