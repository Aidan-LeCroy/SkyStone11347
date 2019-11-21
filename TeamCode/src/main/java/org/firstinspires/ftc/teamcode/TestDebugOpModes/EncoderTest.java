package org.firstinspires.ftc.teamcode.TestDebugOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DiffConstants;
import org.firstinspires.ftc.teamcode.Math.Vector;

@TeleOp(group="Diff Rev1",name="Encoder Telemetry")
public class EncoderTest extends OpMode {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private double leftStick;
    private double rightStick;
    public void init(){
        motor1=hardwareMap.dcMotor.get("topL");
        motor2=hardwareMap.dcMotor.get("bottomL");
        motor3=hardwareMap.dcMotor.get("topR");
        motor4=hardwareMap.dcMotor.get("bottomR");
        DcMotor mot[]={motor1,motor2,motor3,motor4};
        for(int i=0;i<mot.length;i++){

            mot[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            mot[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mot[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void loop(){
        checkSticks();
        drive(leftStick,rightStick);
        log();

    }

    private void log(){
        telemetry.addData("Top Left:",motor1.getCurrentPosition());
        telemetry.addData("Bottom Left:",motor2.getCurrentPosition());
        telemetry.addData("Top Right:",motor3.getCurrentPosition());
        telemetry.addData("Bottom Right:",motor4.getCurrentPosition());
        telemetry.addData("Left Angle",setHeading(motor1,motor2));
        telemetry.addData("Right Angle",setHeading(motor3,motor4));
        Vector lStick=new Vector(gamepad1.left_stick_x,gamepad1.left_stick_y);
        telemetry.addData("1:lstickheading",lStick.getAngle(false));

    }
//    private static final double HEADCONSTANT = (41888.0/192.0);

    private double setHeading(DcMotor topmotor,DcMotor bottommotor){
        double encoderAvg = Math.abs(topmotor.getCurrentPosition() + bottommotor.getCurrentPosition() / 2.0);
        double reciprocal = 1 / DiffConstants.HEADCONSTANT;
        double degreeHeading = ((reciprocal * (encoderAvg % DiffConstants.HEADCONSTANT)) * 360);
        return degreeHeading;
    }
     private void drive(double leftStick,double rightStick){
        motor1.setPower(leftStick);
        motor2.setPower(leftStick);
        motor3.setPower(rightStick);
        motor4.setPower(rightStick);
     }
     private void checkSticks(){
        if(Math.abs(gamepad1.left_stick_y)<= DiffConstants.STICK_DEADZONE)
            leftStick=0;
        else
            leftStick=gamepad1.left_stick_y*.5;
        if(Math.abs(gamepad1.right_stick_y)<= DiffConstants.STICK_DEADZONE)
            rightStick=0;
        else
            rightStick=gamepad1.right_stick_y*.5;
     }

}
