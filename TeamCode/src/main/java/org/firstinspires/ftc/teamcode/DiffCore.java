package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DiffCore extends OpMode {
    DcMotor topL;
    DcMotor topR;
    DcMotor bottomL;
    DcMotor bottomR;
    BNO055IMU imu;

    public static double masterScale=.2;


    public void init(){
        DcMotor motor1=(DcMotor) hardwareMap.get("topL");
        DcMotor motor2=(DcMotor) hardwareMap.get("bottomL");
        DcMotor motor3=(DcMotor) hardwareMap.get("topR");
        DcMotor motor4=(DcMotor) hardwareMap.get("bottomR");


        Cassete leftDrive=new Cassete(motor1,motor2);
        Cassete rightDrive=new Cassete(motor3,motor4);

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
    //when utilizing this method, get the stick position for the two values.
    public void DiffDrive(double stickX,double stickY){

    }


}
