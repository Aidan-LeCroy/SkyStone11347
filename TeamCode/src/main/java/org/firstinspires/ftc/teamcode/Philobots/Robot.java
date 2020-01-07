package org.firstinspires.ftc.teamcode.Philobots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Robot {
    DriveController driveController;
    private BNO055IMU imu;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    private OpMode opMode;

    private DcMotor leftIntake,rightIntake;
    private DcMotor leftLift,rightLift;
    private static final double ENCODER_TICKS_PER_LEVEL=200.0;
    private Servo leftS,rightS,leftSF,rightSF,leftFourB,rightFourB;
    int liftLevel=1;
    public Robot (OpMode opMode, boolean isAuto) {

        this.hardwareMap = opMode.hardwareMap;
        initMechanisms();
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        driveController = new DriveController(this);
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
    }


    public void initIMU () {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }
    public void initMechanisms(){
        leftIntake=hardwareMap.dcMotor.get("leftIntake");
        rightIntake=hardwareMap.dcMotor.get("rightIntake");
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightS=hardwareMap.servo.get("rightS");
        leftS=hardwareMap.servo.get("leftS");
        rightSF=hardwareMap.servo.get("rightSF");
        leftSF=hardwareMap.servo.get("leftSF");
        leftFourB=hardwareMap.servo.get("leftFB");
        rightFourB=hardwareMap.servo.get("rightFB");
        rightLift=hardwareMap.dcMotor.get("rightLift");
        leftLift=hardwareMap.dcMotor.get("leftLift");
    }

    public Angle getRobotHeading () {
        //heading is of NEG_180_TO_180_HEADING type by default (no need for conversion)
        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        telemetry.addData("Robot Heading", heading);
        return new Angle(heading, Angle.AngleType.NEG_180_TO_180_HEADING);
    }

    public void wait (int millis, LinearOpMode linearOpMode) {
        long startTime = System.currentTimeMillis();
        while (millis > System.currentTimeMillis() - startTime && linearOpMode.opModeIsActive()) {}
    }
    public void wait (int millis, OpMode OpMode) {
        long startTime = System.currentTimeMillis();
        while (millis > System.currentTimeMillis() - startTime) {}
    }
    public void dropIntakeServos(){
        leftS.setPosition(.6);
        rightS.setPosition(.2);
    }
    public void intake(double power){
        leftIntake.setPower(-power);
        rightIntake.setPower(power);
    }
    public void dropFoundation(){
        leftSF.setPosition(1);
        rightSF.setPosition(1);
    }
    public void liftFoundation(){
        leftSF.setPosition(0);
        rightSF.setPosition(0);

    }
    public void fourBarPosition(double pos){

    }
    public void activateLift(){
        if(liftLevel*ENCODER_TICKS_PER_LEVEL-100>getLiftEncoder())
            leftLift.setPower(0);
            rightLift.setPower(0);
        if(liftLevel*ENCODER_TICKS_PER_LEVEL>(getLiftEncoder())){
            leftLift.setPower(.5);
            rightLift.setPower(.5);
        }
        if(liftLevel*ENCODER_TICKS_PER_LEVEL<(getLiftEncoder())){
            leftLift.setPower(-.5);
            rightLift.setPower(-.5);
        }
    }
    public void addLevel() {
        liftLevel++;
    }
    public void subLevel(){
        if(liftLevel==1){
            return;
        }
        liftLevel--;
    }
    public int getLevel(){
        return liftLevel;
    }
    public double getLiftEncoder(){
        return ((leftLift.getCurrentPosition()+rightLift.getCurrentPosition())/2.0);
    }




}