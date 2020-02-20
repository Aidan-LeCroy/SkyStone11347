package org.firstinspires.ftc.teamcode.CircuitRunners;

import android.media.MediaPlayer;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveDrive;
import com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveModuleEx;
import com.arcrobotics.ftclib.hardware.motors.MotorImplEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.R;

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

    private PIDFController defaultRevMotorVelo = new PIDFController(new double[] {1.17, 0.117, 0, 11.7});
    private PController defaultRevMotorPos = new PController(5);
    private DiffySwerveModuleEx left, right;
    private DiffySwerveDrive drive;

    private PIDFController leftController = new PIDFController(new double[] {rotationPLeft, 0, 0, 0});
    private PIDFController rightController = new PIDFController(new double[] {rotationPRight, 0, 0, 0});

    private static final String[] driveMotorIds = {"topL", "bottomL", "topR", "bottomR"};

    @Override
    public void runOpMode() throws InterruptedException {

        MediaPlayer mp;
        try {
            mp = MediaPlayer.create(hardwareMap.appContext, R.raw.shutdown);
            mp.start();
        } catch (Exception e) {
            telemetry.addData("Exception Thrown! ", e);
        }

        topLeft = new MotorImplEx(hardwareMap, driveMotorIds[0], driveCPR, defaultRevMotorVelo, defaultRevMotorPos);
        bottomLeft = new MotorImplEx(hardwareMap, driveMotorIds[1], driveCPR, defaultRevMotorVelo, defaultRevMotorPos);
        topRight = new MotorImplEx(hardwareMap, driveMotorIds[2], driveCPR, defaultRevMotorVelo, defaultRevMotorPos);
        bottomRight = new MotorImplEx(hardwareMap, driveMotorIds[3], driveCPR, defaultRevMotorVelo, defaultRevMotorPos);
        left = new DiffySwerveModuleEx(topLeft, bottomLeft, kAngleLeft, kWheelLeft, leftController);
        right = new DiffySwerveModuleEx(topRight, bottomRight, kAngleRight, kWheelRight, rightController);

        left.setHeadingInterpol(() -> AngleUnit.DEGREES.normalize(left.getRawHeading()));
        right.setHeadingInterpol(() -> AngleUnit.DEGREES.normalize(right.getRawHeading()));

        drive = new DiffySwerveDrive(left, right);

        topLeft.resetEncoder();
        topRight.resetEncoder();
        bottomLeft.resetEncoder();
        bottomRight.resetEncoder();

        //drive.setRightSideInverted(true);

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
