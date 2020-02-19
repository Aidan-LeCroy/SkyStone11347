package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveDrive;
import com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveModuleEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorImplEx;
import com.arcrobotics.ftclib.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.CircuitRunners.DriveModule;

public class DriveSubsystem implements Subsystem {

    private DiffySwerveDrive drivebase;
    private DiffySwerveModuleEx leftModule, rightModule;

    private static final String[] driveMotorIds = {"topL", "bottomL", "topR", "bottomR"};
    private static final double driveCPR = 28 * 4 * 15.9;

    private PIDFController defaultRevMotorVelo = new PIDFController(new double[] {1.17, 0.117, 0, 11.7});
    private PController defaultRevMotorPos = new PController(5);

    private double kAngleLeft = DriveModule.DEGREES_PER_TICK;
    private double kAngleRight = DriveModule.DEGREES_PER_TICK;

    private double kWheelLeft = DriveModule.CM_PER_TICK;
    private double kWheelRight = DriveModule.CM_PER_TICK;

    private double rotationPLeft = 1;
    private double rotationPRight = 1;

    private MotorImplEx topLeft, topRight, bottomLeft, bottomRight;

    public enum Direction {
        FORWARD, BACKWARDS, LEFT, RIGHT,
        COUNTERCLOCK, CLOCK
    }

    private LinearOpMode opMode;

    public DriveSubsystem(LinearOpMode opMode) {
        this.opMode = opMode;
        topLeft = new MotorImplEx(
                opMode.hardwareMap, driveMotorIds[0], driveCPR,
                defaultRevMotorVelo, defaultRevMotorPos
        );
        bottomLeft = new MotorImplEx(
                opMode.hardwareMap, driveMotorIds[1], driveCPR,
                defaultRevMotorVelo, defaultRevMotorPos
        );
        topRight = new MotorImplEx(
                opMode.hardwareMap, driveMotorIds[2], driveCPR,
                defaultRevMotorVelo, defaultRevMotorPos
        );
        bottomRight = new MotorImplEx(
                opMode.hardwareMap, driveMotorIds[3], driveCPR,
                defaultRevMotorVelo, defaultRevMotorPos
        );

        leftModule = new DiffySwerveModuleEx(
                topLeft, bottomLeft, kAngleLeft, kWheelLeft,
                new PIDFController(new double[]{rotationPLeft,0,0,0})
        );
        rightModule = new DiffySwerveModuleEx(
                topRight, bottomRight, kAngleRight, kWheelRight,
                new PIDFController(new double[]{rotationPRight,0,0,0})
        );

        topLeft.resetEncoder();
        bottomLeft.resetEncoder();
        topRight.resetEncoder();
        bottomRight.resetEncoder();
        topLeft.setMode(MotorEx.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(MotorEx.RunMode.RUN_USING_ENCODER);
        topRight.setMode(MotorEx.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(MotorEx.RunMode.RUN_USING_ENCODER);

        leftModule.setHeadingInterpol(
                () -> AngleUnit.DEGREES.normalize(leftModule.getRawHeading())
        );
        rightModule.setHeadingInterpol(
                () -> AngleUnit.DEGREES.normalize(rightModule.getRawHeading())
        );

        drivebase = new DiffySwerveDrive(leftModule, rightModule);
    }

    @Override
    public void initialize(){

        stop();
    }

    public void driveInDirection(Direction direction, double power) throws Exception {
        if (direction == Direction.BACKWARDS)
            drivebase.drive(
                    new Vector2d(0, -power),
                    new Vector2d(0, -power)
            );
        else if (direction == Direction.FORWARD)
            drivebase.drive(
                    new Vector2d(0, power),
                    new Vector2d(0, power)
            );
        else if (direction == Direction.LEFT) {
            drivebase.drive(
                    new Vector2d(-power,0),
                    new Vector2d(-power,0)
            );
        }
        else if (direction == Direction.RIGHT) {
            drivebase.drive(
                    new Vector2d(power,0),
                    new Vector2d(power,0)
            );
        }
        else if (direction == Direction.CLOCK) {
            drivebase.drive(
                    new Vector2d(0, power),
                    new Vector2d(0, -power)
            );
        }
        else if (direction == Direction.COUNTERCLOCK) {
            drivebase.drive(
                    new Vector2d(0, -power),
                    new Vector2d(0, power)
            );
        } else {
            throw new Exception("Unspecified Direction!");
        }
    }

    @Override
    public void stop(){
        drivebase.stopMotor();
    }
}
