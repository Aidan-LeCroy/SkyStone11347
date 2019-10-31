package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Math.Vector;

@TeleOp(name="core1.1",group="Diff")

public class DiffCore extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //top left and bottom right motors bad.
    private DcMotor motor1,motor2,motor3,motor4;
    private DcMotor leftIntake, rightIntake;
    private Cassete leftDrive;
    private Cassete rightDrive;
    private String scaleString;

    static double masterScale=.2;

    public void init(){
        motor1=hardwareMap.dcMotor.get("topL");
        motor2=hardwareMap.dcMotor.get("bottomL");
        motor3=hardwareMap.dcMotor.get("topR");
        motor4=hardwareMap.dcMotor.get("bottomR");

        leftIntake=hardwareMap.dcMotor.get("intakeL");
        rightIntake=hardwareMap.dcMotor.get("intakeR");


        gamepad1.setJoystickDeadzone(.1f);
        slowMode();
        leftDrive=new Cassete(motor1,motor2,Math.toRadians(180),"LEFTDRIVE");
        rightDrive=new Cassete(motor3,motor4,Math.toRadians(0),"RIGHTDRIVE");
//        resetEncoders();

    }
    public void loop(){
        //main loop
        diffDrive(gamepad1.left_stick_x,gamepad1.left_stick_y);
        logMotorStats();
        leftDrive.robotLog();
        rightDrive.robotLog();


//        update();

    }
    public void start(){
        runtime.reset();
        leftDrive.resetRuntime();
        rightDrive.resetRuntime();
    }




//WIP
    public void diffDrive(double stickLX,double stickLY) {
        Vector direction = new Vector(stickLX, stickLY);
        double wheelMagnitude = direction.getMagnitude();
        double wheelAngle = direction.getAngle(true);
        leftDrive.update();
        rightDrive.update();
    }

    public void resetEncoders(){
        leftDrive.resetEncoders();
        rightDrive.resetEncoders();
    }
    public void logMotorStats(){
        telemetry.addData("Left Drive: ",leftDrive.getLogString());
        telemetry.addData("Right Drive: ",rightDrive.getLogString());
        telemetry.addData("Mode: ",scaleString);
    }



    public void fastMode() {
        masterScale = 0.7;
        scaleString="i am speed";
    }
    public void slowMode(){
        masterScale = 0.2;
        scaleString="i am not speed";
    }
    //return true if input is detected on either stick
    //false if otherwise
    private boolean checkInputs(){
        if(gamepad1.left_stick_x>.05||gamepad1.left_stick_y>.05||gamepad1.right_stick_x>.05||gamepad1.right_stick_y>.05)
            return true;
        else
            return false;
    }
    private void intake(){
//        leftIntake.setPower(Range.clip(100*gamepad1.left_trigger,0,1));
//        rightIntake.setPower(-Range.clip(100*gamepad1.right_trigger,-1,0));
        double rightScalar=1;
        double leftScalar=1;


        if(gamepad1.right_bumper){
            rightScalar=-1;
        }
        if(gamepad1.left_bumper){
            leftScalar=-1;
        }

        leftIntake.setPower(-gamepad1.left_trigger*leftScalar);
        rightIntake.setPower(gamepad1.right_trigger*rightScalar);
    }


}
