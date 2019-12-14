package org.firstinspires.ftc.teamcode.Philobots;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Philobots.Angle;

@Autonomous(name = "Diff Swerve Test Auto", group = "Linear Opmode")

public class TestAuto extends LinearOpMode {
    Robot robot;

    public void runOpMode() {
        robot = new Robot(this, true);
        robot.initIMU();

        //simple sequence to demonstrate the three main autonomous primitives

        //rotate modules to face to the right
        robot.driveController.rotateModules(Vector2d.RIGHT, 4000, this);

        //drive 20 cm to the right (while facing forward)
        robot.driveController.drive(Vector2d.RIGHT, 20, 1, this,2000);

        //turn to face robot right
        robot.driveController.rotateRobot(Angle.RIGHT, this);
    }
}

