package org.firstinspires.ftc.teamcode.CircuitRunners;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveDrive;
import com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveModuleEx;
import com.arcrobotics.ftclib.hardware.motors.MotorImplEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Kill Me Please")
public class ModuleTesting extends LinearOpMode {

    MotorImplEx topLeft, bottomRight, topRight, bottomLeft;

    private double kWheelLeft = DriveModule.CM_PER_TICK;
    private double kWheelRight = DriveModule.CM_PER_TICK;

    private double kAngleLeft = DriveModule.DEGREES_PER_TICK;
    private double kAngleRight = DriveModule.DEGREES_PER_TICK;

    private double rotationPLeft = 1;
    private double rotationPRight = 1;

    private static final double driveCPR = 28 * 4 * 15.9;

    PIDFController defaultRevMotorVelo = new PIDFController(new double[] {1.17, 0.117, 0, 11.7});
    PController defaultRevMotorPos = new PController(5);

    DiffySwerveModuleEx left, right;

    DiffySwerveDrive drive;

    private static final String[] driveMotorIds = {"topL", "bottomL", "topR", "bottomR"};

    @Override
    public void runOpMode() throws InterruptedException {

        topLeft = new MotorImplEx(hardwareMap, driveMotorIds[0], driveCPR, defaultRevMotorVelo, defaultRevMotorPos);
        bottomLeft = new MotorImplEx(hardwareMap, driveMotorIds[1], driveCPR, defaultRevMotorVelo, defaultRevMotorPos);
        topRight = new MotorImplEx(hardwareMap, driveMotorIds[2], driveCPR, defaultRevMotorVelo, defaultRevMotorPos);
        bottomRight = new MotorImplEx(hardwareMap, driveMotorIds[3], driveCPR, defaultRevMotorVelo, defaultRevMotorPos);

        left = new DiffySwerveModuleEx(topLeft, bottomLeft, kAngleLeft, kWheelLeft, rotationPLeft);
        right = new DiffySwerveModuleEx(topRight, bottomRight, kAngleRight, kWheelRight, rotationPRight);

        left.setHeadingInterpol(() -> AngleUnit.DEGREES.normalize(left.getRawHeading()));
        right.setHeadingInterpol(() -> AngleUnit.DEGREES.normalize(right.getRawHeading()));

        drive = new DiffySwerveDrive(left, right);

        topLeft.resetEncoder();
        topRight.resetEncoder();
        bottomLeft.resetEncoder();
        bottomRight.resetEncoder();

        drive.setRightSideInverted(true);

        waitForStart();

        while (opModeIsActive()) {
            drive.drive(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    -gamepad1.right_stick_y
            );

            telemetry.addData("Left Heading", left.moduleHeading.getAsDouble());
            telemetry.addData("Right Heading", right.moduleHeading.getAsDouble());
            telemetry.addData("kAngle", kAngleLeft);
            telemetry.update();
        }

    }

}
