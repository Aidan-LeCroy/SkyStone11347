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
import org.firstinspires.ftc.teamcode.Philobots.Angle;

public class Robot {
    DriveController driveController;
    BNO055IMU imu;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    OpMode opMode;

    private DcMotor leftIntake,rightIntake;
    private Servo leftS,rightS,leftSF,rightSF;
    public Robot (OpMode opMode, boolean isAuto) {

        this.hardwareMap = opMode.hardwareMap;
        leftIntake=hardwareMap.dcMotor.get("leftIntake");
        rightIntake=hardwareMap.dcMotor.get("rightIntake");
        rightS=hardwareMap.servo.get("rightS");
        leftS=hardwareMap.servo.get("leftS");
        rightSF=hardwareMap.servo.get("rightSF");
        leftSF=hardwareMap.servo.get("leftSF");
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
    private void dropIntakeServos(){
        leftS.setPosition(.4);
        rightS.setPosition(.4);
    }
    private void intake(double power){
        leftIntake.setPower(-power);
        rightIntake.setPower(power);
    }
    public void dropFoundation(){
        leftSF.setPosition(.8);
        rightSF.setPosition(.8);
    }
    public void liftFoundation(){
        leftSF.setPosition(0);
        rightSF.setPosition(0);

    }
}