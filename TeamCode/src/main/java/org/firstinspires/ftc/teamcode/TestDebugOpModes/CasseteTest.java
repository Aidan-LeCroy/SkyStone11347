package org.firstinspires.ftc.teamcode.TestDebugOpModes;

import com.qualcomm.hardware.bosch.BNO055IMU; // Bosch BNO055 Inertial Motion Unit, aka the robot's eyes
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="coreTest",group="Diff")
class CasseteTest extends OpMode {



    public CasseteTest () {}

    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;
    //top left and bottom right motors bad.
    private DcMotor motor1,motor2,motor3,motor4;
    private DcMotor leftIntake, rightIntake;


    public void init(){
        motor1=hardwareMap.dcMotor.get("topL");
        motor2=hardwareMap.dcMotor.get("bottomL");
        motor3=hardwareMap.dcMotor.get("topR");
        motor4=hardwareMap.dcMotor.get("bottomR");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);

//        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    public void loop(){

        motor1.setPower(.5*gamepad1.left_stick_y);
        motor2.setPower(.66*gamepad1.left_stick_y);
        motor3.setPower(.79*gamepad1.right_stick_y);
        motor4.setPower(.675*gamepad1.right_stick_y);

    }
    public void start(){
        runtime.reset();
    }

    void rightDrive(double power) {

    }

}
