package org.firstinspires.ftc.teamcode.TestDebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CircuitRunners.DriveController;
import org.firstinspires.ftc.teamcode.CircuitRunners.Robot;
@TeleOp(name="HoldXToZero",group="Test")
public class WheelZeroing extends OpMode {
    Robot robot;
    DriveController drive;
    public void init(){
        robot=new Robot(this,false,false);
        drive=robot.driveController;
    }
    public void loop(){
        telemetry.addData("Left Heading",robot.driveController.moduleLeft.getCurrentOrientation().getAngle());
        telemetry.addData("Right Heading",robot.driveController.moduleRight.getCurrentOrientation().getAngle());
        if(gamepad1.x) {
            drive.applyPowersToZero();

        }
    }

    public void stop(){

    }
}
