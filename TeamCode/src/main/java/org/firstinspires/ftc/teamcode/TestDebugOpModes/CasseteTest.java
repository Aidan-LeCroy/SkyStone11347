package org.firstinspires.ftc.teamcode.TestDebugOpModes;

import com.qualcomm.hardware.bosch.BNO055IMU; // Bosch BNO055 Inertial Motion Unit, aka the robot's eyes
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="coreTest",group="Diff")

public class CasseteTest extends OpMode {

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
    }
    public void loop(){
        if(gamepad1.a) {
            motor4.setPower(.5);
        }
        else if(gamepad1.b) {
            motor3.setPower(.5);
        }
        else if(gamepad1.x) {
            motor2.setPower(.5);
        }
        else if(gamepad1.y) {
            motor1.setPower(.5);
        }
    }
    public void start(){
        runtime.reset();
    }

    void rightDrive(double power) {

    }

}
