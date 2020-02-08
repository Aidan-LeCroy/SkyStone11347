package org.firstinspires.ftc.teamcode.TestDebugOpModes;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveModuleEx;
import com.arcrobotics.ftclib.hardware.motors.MotorImplEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import  com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveDrive;

import org.firstinspires.ftc.teamcode.CircuitRunners.DriveModule;

public class FTCLibDiffyControl extends LinearOpMode {

    private DiffySwerveDrive diffySwerveDrive; //Drive base object

    private MotorImplEx topLeft, bottomLeft, topRight, bottomRight;

    private DiffySwerveModuleEx moduleLeft, moduleRight;


    PIDFController pidLeft = new PIDFController(new double[] {0,0,0});
    PIDFController pidRight = new PIDFController(new double[] {0,0,0});


    @Override
    public void runOpMode(){

        moduleLeft = new DiffySwerveModuleEx(topLeft,bottomLeft, 0.8, DriveModule.CM_PER_TICK);
        moduleRight = new DiffySwerveModuleEx(topRight,bottomRight, 0.8, DriveModule.CM_PER_TICK);

        diffySwerveDrive = new DiffySwerveDrive(moduleLeft, moduleRight);

        diffySwerveDrive.setRightSideInverted(true);
        diffySwerveDrive.stopMotor();

        waitForStart();

        while (opModeIsActive()){

            if(!gamepad1.left_bumper) {
                diffySwerveDrive.drive(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.right_stick_y
                );
            } else {
                diffySwerveDrive.drive(
                        gamepad1.left_stick_x * 0.75,
                        -gamepad1.left_stick_y * 0.75,
                        gamepad1.right_stick_x * 0.75,
                        gamepad1.right_stick_y * 0.75
                );
            }
        }
        diffySwerveDrive.stopMotor();
        diffySwerveDrive.disable();






    }
}
