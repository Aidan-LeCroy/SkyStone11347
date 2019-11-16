package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="GK-Core", group="GK")
@Disabled
public class GKCore extends OpMode {

    //    Motors and Servos:
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

    public GKCore() {}

    @Override
    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        frontL = hardwareMap.get(DcMotor.class, "frontL");
        frontL.setDirection(DcMotor.Direction.REVERSE);
        frontR = hardwareMap.get(DcMotor.class, "frontR");
        backL = hardwareMap.get(DcMotor.class, "backL");
        backL.setDirection(DcMotor.Direction.REVERSE);
        backR = hardwareMap.get(DcMotor.class, "backR");
        flip = hardwareMap.get(DcMotor.class, "flip");
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        intakeR = hardwareMap.get(DcMotor.class, "intakeR");
        intakeR.setDirection(DcMotor.Direction.REVERSE);

        backL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {
        runtime.reset();
        super.start();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        super.stop();
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

    public void turn(double ang, double time) {
        checkAngle();
//        makes a cutoff time to complete the turn
        double sTime = getRuntime();
        double fTime = getRuntime();
        while (Math.abs(ang) - Math.abs(curAng.firstAngle) > 3 && sTime + 5 < fTime){
            if (ang - curAng.firstAngle > 25) {
                backR.setPower(.3);
                frontR.setPower(.3);
                backL.setPower(-.3);
                frontL.setPower(-.3);
            } else if (ang - curAng.firstAngle < 25 && ang - curAng.firstAngle > 0) {
                backR.setPower(.15);
                frontR.setPower(.15);
                backL.setPower(-.15);
                frontL.setPower(-.15);
            } else if (ang - curAng.firstAngle < 0 && ang - curAng.firstAngle > -25) {
                backR.setPower(-.15);
                frontR.setPower(-.15);
                backL.setPower(.15);
                frontL.setPower(.15);
            } else if (ang - curAng.firstAngle < -25) {
                backR.setPower(-.3);
                frontR.setPower(-.3);
                backL.setPower(.3);
                frontL.setPower(.3);
            }
            fTime = getRuntime();
            checkAngle();
        }
    }

    public void autoDrive(int dist, double power) {
        int tick = (int)((dist * 2 * Math.PI) / (288 * 45 * 25.4));

        resetEncoders();

        backL.setTargetPosition(tick);
        backR.setTargetPosition(tick);
        frontL.setTargetPosition(tick);
        frontR.setTargetPosition(tick);

        backL.setPower(power);
        backR.setPower(power);
        frontL.setPower(power);
        frontR.setPower(power);
    }

}