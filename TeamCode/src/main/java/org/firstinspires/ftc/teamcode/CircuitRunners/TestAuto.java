package org.firstinspires.ftc.teamcode.CircuitRunners;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Diff Swerve Test Auto", group = "Linear Opmode")
class TestAuto extends LinearOpMode {
    private Robot robot;
    public void runOpMode() {
        robot = new Robot(this, true);
        robot.initIMU();

        //simple sequence to demonstrate the three main autonomous primitives

        //rotate modules to face to the right
        robot.driveController.rotateModules(Vector2d.RIGHT, this);

        //drive 20 cm to the right (while facing forward)
        robot.driveController.drive(Vector2d.RIGHT, 20, 1, this);

        //turn to face robot right
        robot.driveController.rotateRobot(Angle.RIGHT, this);
    }
}

