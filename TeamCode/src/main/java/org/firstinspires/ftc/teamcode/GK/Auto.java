package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="GK auto", group="GK")
public class Auto extends LinearOpMode {

    public DcMotor frontL;
    public DcMotor frontR;
    public DcMotor backL;
    public DcMotor backR;
    public DcMotor flip;
    public DcMotor intakeR;
    public DcMotor intakeL;

    //    Sensors:
    public BNO055IMU imu;

    //    Variables:
    public Orientation curAng;
    public ElapsedTime runtime = new ElapsedTime();
    final static int ticks = (int)((288*25.4) / (2.4*90*Math.PI));

    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        frontL = hardwareMap.get(DcMotor.class, "frontL");
        frontR = hardwareMap.get(DcMotor.class, "frontR");
        frontR.setDirection(DcMotor.Direction.REVERSE);
        backL = hardwareMap.get(DcMotor.class, "backL");
        backR = hardwareMap.get(DcMotor.class, "backR");
        backR.setDirection(DcMotor.Direction.REVERSE);
        flip = hardwareMap.get(DcMotor.class, "flip");
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        intakeR = hardwareMap.get(DcMotor.class, "intakeR");

        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        autoDrive(12, .5, 3);


    }

    public void turn(double ang, double time) {
        checkAngle();
//        makes a cutoff time to complete the turn
        double sTime = getRuntime();
        double fTime = getRuntime();
        while (Math.abs(ang) - Math.abs(curAng.firstAngle) > 3 && sTime + (time * 1000) > fTime){
            if (ang - curAng.firstAngle > 25) {
                backR.setPower(.65);
                frontR.setPower(.65);
                backL.setPower(-.65);
                frontL.setPower(-.65);
            } else if (ang - curAng.firstAngle < 25 && ang - curAng.firstAngle > 0) {
                backR.setPower(.5);
                frontR.setPower(.5);
                backL.setPower(-.5);
                frontL.setPower(-.5);
            } else if (ang - curAng.firstAngle < 0 && ang - curAng.firstAngle > -25) {
                backR.setPower(-.5);
                frontR.setPower(-.5);
                backL.setPower(.5);
                frontL.setPower(.5);
            } else if (ang - curAng.firstAngle < -25) {
                backR.setPower(-.65);
                frontR.setPower(-.65);
                backL.setPower(.65);
                frontL.setPower(.65);
            }
            fTime = getRuntime();
            checkAngle();
        }
        backR.setPower(0);
        frontR.setPower(0);
        backL.setPower(0);
        frontL.setPower(0);
    }

    public void autoDrive(int dist, double power, double time) {
        resetEncoders();

        backL.setTargetPosition(dist*ticks);
        backR.setTargetPosition(dist*ticks);
        frontL.setTargetPosition(dist*ticks);
        frontR.setTargetPosition(dist*ticks);
        double sTime = getRuntime();
        double fTime = getRuntime();
        while (sTime + time >= fTime) {
            backL.setPower(power);
            backR.setPower(power);
            frontL.setPower(power);
            frontR.setPower(power);
            fTime = getRuntime();

        }
        backL.setPower(0);
        backR.setPower(0);
        frontL.setPower(0);
        frontR.setPower(0);
    }

    public void checkAngle() {
        curAng = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    public void resetEncoders() {
        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
