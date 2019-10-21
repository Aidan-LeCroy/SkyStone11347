package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU; // Bosch BNO055 Inertial Motion Unit, aka the robot's eyes
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="core1.1",group="Diff")

public class DiffCore extends OpMode {

    BNO055IMU imu;
    private DcMotor motor1,motor2,motor3,motor4;
    private Cassete leftDrive;
    private Cassete rightDrive;

    private double forwardsPower = 0;
    private double sidewaysPower = 0;
    private double amountTurn = 0;

    private String scaleString;

    public static double WHEEL_CIRCUMFERENCE = 90 * Math.PI, RATIO = 4, RPM, LENGTH, WIDTH, HEIGHT; // change as design changes, use mm
    public static double masterScale=.2;

    public void init(){
        motor1=hardwareMap.dcMotor.get("topL");
        motor2=hardwareMap.dcMotor.get("bottomL");
        motor3=hardwareMap.dcMotor.get("topR");
        motor4=hardwareMap.dcMotor.get("bottomR");


        leftDrive=new Cassete(motor1,motor2,Math.toRadians(180));
        rightDrive=new Cassete(motor3,motor4,Math.toRadians(0));


    }
    public void loop(){
        //main loop
        DiffDrive(gamepad1.left_stick_x,gamepad1.left_stick_y);
        logMotorStats();
//        DiffTurn(gamepad1.right_stick_x);
        update();

    }




    public void DiffAutoDrive(double angle, double power){

    }
    //when utilizing this method, get the stick position for the two values. Might move this method to DiffTeleOp, not quite sure yet, still figuring it out
    //TODO: Test this, no clue if it works.

    public void DiffDrive(double stickXL,double stickYL){
        Vector direction = new Vector(stickXL, stickYL);
        setForwardsPower(direction.getMagnitude());
//        setSidewaysPower();
        double angle1=direction.getAngle();
//        setamountTurn(angle1);

    }

    public void update(){
        rightDrive.setDriveTrainDirection(forwardsPower,sidewaysPower,amountTurn);
        leftDrive.setDriveTrainDirection(forwardsPower,sidewaysPower,amountTurn);

        //update the modules
        rightDrive.update();
        leftDrive.update();
    }
    public void setSidewaysPower(double power){
        sidewaysPower = power;
    }
    public void setForwardsPower(double power){
        forwardsPower = power;
    }
    public void setamountTurn(double amount){
        amountTurn = amount;
    }
    public void resetEncoders(){
        leftDrive.resetEncoders();
        rightDrive.resetEncoders();
    }
    public void logMotorStats(){
        telemetry.addData("Left Drive: ",leftDrive.getLogString());
        telemetry.addData("Right Drive: ",rightDrive.getLogString());

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
