package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU; // Bosch BNO055 Inertial Motion Unit, aka the robot's eyes
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Math.Vector;
public class DiffCore extends OpMode {
    DcMotor topL;
    DcMotor topR;
    DcMotor bottomL;
    DcMotor bottomR;
    BNO055IMU imu;

    Cassete leftDrive;
    Cassete rightDrive;

    private double forwardsPower = 0;
    private double sidewaysPower = 0;
    private double amountTurn = 0;

    private String scaleString;

    public static double WHEEL_CIRCUMFERENCE = 90 * Math.PI, RATIO = 4, RPM, LENGTH, WIDTH, HEIGHT; // change as design changes, use mm
    public static double masterScale=.2;

    public void init(){
        DcMotor motor1=hardwareMap.dcMotor.get("topL");
        DcMotor motor2=hardwareMap.dcMotor.get("bottomL");
        DcMotor motor3=hardwareMap.dcMotor.get("topR");
        DcMotor motor4=hardwareMap.dcMotor.get("bottomR");


        leftDrive=new Cassete(motor1,motor2);
        rightDrive=new Cassete(motor3,motor4);

        DcMotor[] motors={topL,topR,bottomL,bottomR};
        for(DcMotor mot:motors) {
            mot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void loop(){
        // Not sure what to loop yet, probably debug stuff using RobotLog and telemetry updates, since this is not the main loop.

    }




    public void DiffAutoDrive(double angle, double power){

    }
    //when utilizing this method, get the stick position for the two values. Might move this method to TeleOp, not quite sure yet, still figuring it out
//    public void DiffDrive(double stickXL,double stickYL, double stickXR, double stickYR){
//        Vector direction = new Vector(stickXL, stickYL);
//        Vector rotation = new Vector(stickXR, stickYR);
//        Vector dirrot = direction.add(rotation).scale(masterScale);
//
//    }
    public void update(){
        rightDrive.setDriveTrainDirection(forwardsPower,sidewaysPower,amountTurn);
        leftDrive.setDriveTrainDirection(forwardsPower,sidewaysPower,amountTurn);

        //update the modules
        rightDrive.update();
        leftDrive.update();
    }

    public void fastMode() {
        masterScale = 0.7;
        scaleString="i am speed";
    }
    public void slowMode(){
        masterScale = 0.2;
        scaleString="i am not speed";
    }

}
