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

    public void init(){
        topL= hardwareMap.dcMotor.get("topL");
        topR= hardwareMap.dcMotor.get("topR");
        bottomL= hardwareMap.dcMotor.get("bottomL");
        bottomR= hardwareMap.dcMotor.get("bottomR");
        DcMotor[] motors={topL,topR,bottomL,bottomR};
        for(DcMotor mot:motors) {
            mot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
