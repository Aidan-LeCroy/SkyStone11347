package org.firstinspires.ftc.teamcode.TestDebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(group="Diff Rev1",name="Encoder Telemetry")
public class EncoderTest extends OpMode {
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    double leftStick;
    double rightStick;
    final double  STICK_DEADZONE=.2;
    public void init(){
        motor1=hardwareMap.dcMotor.get("topL");
        motor2=hardwareMap.dcMotor.get("bottomL");
        motor3=hardwareMap.dcMotor.get("topR");
        motor4=hardwareMap.dcMotor.get("bottomR");
        DcMotor mot[]={motor1,motor2,motor3,motor4};
        for(int i=0;i<mot.length;i++){
            mot[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            mot[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void loop(){
        drive(leftStick,rightStick);
        log();
        checkSticks();
    }

    private void log(){
        telemetry.addData("Top Left:",motor1.getCurrentPosition());
        telemetry.addData("Bottom Left:",motor2.getCurrentPosition());
        telemetry.addData("Top Right:",motor3.getCurrentPosition());
        telemetry.addData("Bottom Right:",motor4.getCurrentPosition());
    }
     private void drive(double leftStick,double rightStick){
        motor1.setPower(leftStick);
        motor2.setPower(leftStick);
        motor3.setPower(rightStick);
        motor4.setPower(rightStick);
     }
     private void checkSticks(){
        if(Math.abs(gamepad1.left_stick_y)<=STICK_DEADZONE)
            leftStick=0;
        else
            leftStick=gamepad1.left_stick_y;
        if(Math.abs(gamepad1.right_stick_y)<=STICK_DEADZONE)
            rightStick=0;
        else
            rightStick=gamepad1.right_stick_y;
     }

}
