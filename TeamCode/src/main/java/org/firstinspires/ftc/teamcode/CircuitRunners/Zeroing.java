package org.firstinspires.ftc.teamcode.CircuitRunners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Zero",group="TeleOp")
public class Zeroing extends OpMode {
    Robot robot = new Robot(this, false);

    public void init() {
        robot.initMechanisms();
    }

    public void stop() {
        robot.driveController.resetEncoders();
    }

    public void loop() {
        robot.driveController.moduleLeft.directDrive(-gamepad1.left_stick_y);
        robot.driveController.moduleRight.directDrive(-gamepad1.right_stick_y);
    }
}