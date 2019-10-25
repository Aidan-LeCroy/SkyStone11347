package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


//lol why did I make this
@TeleOp(group="zippy",name="TeleOp")
public class ZippyTeleOp extends OpMode {
    DcMotor leftDrive,rightDrive,leftIntake,rightIntake;

    public void init(){
        leftDrive=hardwareMap.dcMotor.get("leftDrive");
        rightDrive=hardwareMap.dcMotor.get("rightDrive");
        leftIntake=hardwareMap.dcMotor.get("leftIntake");
        rightIntake=hardwareMap.dcMotor.get("rightIntake");
    }
    public void loop(){
    intake();
    zippyDrive();
    log();
    }
    private void intake(){
        double leftPower=gamepad1.left_trigger;
        double rightPower=gamepad1.right_trigger;
        if(gamepad1.left_bumper)
            leftPower*=-1;
        if(gamepad1.right_bumper)
            rightPower*=-1;
        leftIntake.setPower(-leftPower);
        rightIntake.setPower(rightPower);
    }
    private void zippyDrive(){
        leftDrive.setPower(-gamepad1.left_stick_y);
        rightDrive.setPower(-gamepad1.right_stick_y);
    }

    private void log(){
        telemetry.addData("zipp: ","leftEncoder: "+leftDrive.getCurrentPosition()+"rightEncoder: "+rightDrive.getCurrentPosition());
    }
}
