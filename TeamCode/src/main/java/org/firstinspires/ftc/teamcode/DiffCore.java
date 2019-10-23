package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU; // Bosch BNO055 Inertial Motion Unit, aka the robot's eyes
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="core1.1",group="Diff")

public class DiffCore extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;
    private DcMotor motor1,motor2,motor3,motor4;
    private DcMotor leftIntake, rightIntake;
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
//        DiffDrive(gamepad1.left_stick_x,gamepad1.left_stick_y);
        DiffTankDrive();
//        DiffTankTurn();
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
//    public void DiffTankTurn(){
//        leftDrive.setDrivePower(gamepad1.right_stick_x);
//        rightDrive.setDrivePower(-gamepad1.right_stick_x);
//    }



    public void DiffAutoDrive(double angle, double power){

    }
    //TODO: Code does not work
//WIP
//    public void DiffDrive(double stickLX,double stickLY){
//        Vector direction = new Vector(stickLX, stickLY);
//        setforwardsPower(direction.getMagnitude());
////        setSidewaysPower();
//        double angle1=direction.getAngle();
////        setamountTurn(angle1);
//
//    }
    public void DiffTankDrive(){
        double leftPower;
        double rightPower;
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        leftDrive.setDrivePower(leftPower);
        rightDrive.setDrivePower(rightPower);

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
    public void setforwardsPower(double power){
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
        if(gamepad1.right_trigger>.1) {
            leftIntake.setPower(1);
            rightIntake.setPower(-1);
        }
        else if(gamepad1.left_trigger>.1){
            leftIntake.setPower(-1);
            rightIntake.setPower(1);
        }

    }

}
