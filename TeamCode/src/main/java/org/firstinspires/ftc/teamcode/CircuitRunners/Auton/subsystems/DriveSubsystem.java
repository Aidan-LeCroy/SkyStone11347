package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveDrive;
import com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveModuleEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorImplEx;
import com.arcrobotics.ftclib.geometry.Vector2d;

import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.CircuitRunners.DriveModule;

public class DriveSubsystem implements Subsystem {

    private DiffySwerveDrive drivebase;
    private DiffySwerveModuleEx leftModule, rightModule;

    private static final String[] driveMotorIds = {"topL", "bottomL", "topR", "bottomR"};
    private static final double driveCPR = 28 * 4 * 15.9;

    public static final double TRACK_WIDTH = 14.5;

    private RevIMU imu;

    private PIDFController defaultRevMotorVelo = new PIDFController(new double[] {1.17, 0.117, 0, 11.7});
    private PController defaultRevMotorPos = new PController(5);

    private double kAngleLeft = DriveModule.DEGREES_PER_TICK;
    private double kAngleRight = DriveModule.DEGREES_PER_TICK;

    private double kWheelLeft = DriveModule.CM_PER_TICK;
    private double kWheelRight = DriveModule.CM_PER_TICK;

    private double rotationPLeft = 1;
    private double rotationPRight = 1;

    private MotorImplEx topLeft, topRight, bottomLeft, bottomRight;

    private DifferentialOdometry odometry;

    public enum Direction {
        FORWARD, BACKWARDS, LEFT, RIGHT,
        COUNTERCLOCK, CLOCK, STOP
    }

    public DriveSubsystem(LinearOpMode opMode) {
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

        imu = new RevIMU(opMode.hardwareMap, "imu");
    }

    @Override
    public void initialize(){
        leftModule = new DiffySwerveModuleEx(
                topLeft, bottomLeft, kAngleLeft, kWheelLeft,
                new PIDFController(new double[]{rotationPLeft,0,0,0})
        );
        rightModule = new DiffySwerveModuleEx(
                topRight, bottomRight, kAngleRight, kWheelRight,
                new PIDFController(new double[]{rotationPRight,0,0,0})
        );

        leftModule.setHeadingInterpol(
                () -> AngleUnit.DEGREES.normalize(leftModule.getRawHeading())
        );
        rightModule.setHeadingInterpol(
                () -> AngleUnit.DEGREES.normalize(rightModule.getRawHeading())
        );

        drivebase = new DiffySwerveDrive(leftModule, rightModule);

        odometry = new DifferentialOdometry(TRACK_WIDTH);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.init(parameters);

        stop();
    }

    public double[] getModulePositions() {
        leftModule.updateTracking();
        rightModule.updateTracking();
        return new double[]{leftModule.getDistanceTravelled(), rightModule.getDistanceTravelled()};
    }

    public boolean atPosition(Pose2d position) {
        odometry.update(imu.getHeading(), getModulePositions()[0], getModulePositions()[1]);
        return odometry.robotPose.equals(position);
    }

    public Translation2d getTranslation() {
        return odometry.robotPose.getTranslation();
    }

    public double getRotation() {
        return odometry.robotPose.getHeading();
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
        } else if (direction == Direction.STOP) {
            stop();
        }
        else {
            stop();
            //this should never happen...
            throw new Exception("Unspecified Direction!");
        }
    }

    @Override
    public void stop(){
        drivebase.stopMotor();
    }
}
