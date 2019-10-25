package org.firstinspires.ftc.teamcode.TestDebugOpModes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Cassete;
public class GetRPM extends OpMode {
    //find RPM, rotate each cassete
    DcMotor motor1,motor2,motor3,motor4;


    public void init(){
        motor1=hardwareMap.dcMotor.get("topL");
        motor2=hardwareMap.dcMotor.get("bottomL");
        motor3=hardwareMap.dcMotor.get("topR");
        motor4=hardwareMap.dcMotor.get("bottomR");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);

//        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    public void loop(){
        
    }
}
