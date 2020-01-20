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
    private int liftLevel=1;
    private DcMotor leftIntake,rightIntake;
    private DcMotor leftLift,rightLift;
    private Servo leftS,rightS,leftSF,rightSF, leftFB, rightFB,grab;
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
    public void manLift(double power){
        leftLift.setPower(-power);
        rightLift.setPower(power);
    }
    public void initMechanisms(){
        leftIntake=hardwareMap.dcMotor.get("leftIntake");
        rightIntake=hardwareMap.dcMotor.get("rightIntake");
        grab=hardwareMap.servo.get("grabber");
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFB=hardwareMap.servo.get("leftFB");
        rightFB=hardwareMap.servo.get("rightFB");
        rightFB.setDirection(Servo.Direction.REVERSE);
        rightLift=hardwareMap.dcMotor.get("rightLift");
        leftLift=hardwareMap.dcMotor.get("leftLift");
        rightFB.setPosition(.95);
        leftFB.setPosition(.95);
        grab.setPosition(.9);
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
    public void wait (int millis) {
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
    public void setV4BPosition(double pos){

    }
    public void addLevel(){
        liftLevel++;
    }
    public void removeLevel(){
        if(liftLevel==1){
            telemetry.addData("Lift Level: ","Cannot go below level 1");
            return;
        }
        liftLevel--;
    }
    public void setLevel(int level){
        liftLevel=level;
    }
    public int getLiftLevel(){
        return liftLevel;
}
    public double getLiftEncoders(){
        return ((leftLift.getCurrentPosition()+rightLift.getCurrentPosition())/2.0);
    }
}