package org.firstinspires.ftc.teamcode.CircuitRunners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

enum ModuleSide {LEFT, RIGHT}

public class DriveController {
    Robot robot;

    DriveModule moduleLeft;
    DriveModule moduleRight;

    //used for straight line distance tracking
    private double robotDistanceTraveled = 0;
    private double previousRobotDistanceTraveled = 0;
    private double moduleLeftLastDistance;
    private double moduleRightLastDistance;

    //tolerance for module rotation (in degrees)
    public final double ALLOWED_MODULE_ROT_ERROR = 5;

    //distance from target when power scaling will begin
    public final double START_DRIVE_SLOWDOWN_AT_CM = 15;

    //maximum number of times the robot will try to correct its heading when rotating
    public final int MAX_ITERATIONS_ROBOT_ROTATE = 3;

    //will multiply the input from the rotation joystick (max value of 1) by this factor
    public final double ROBOT_ROTATION_SCALE_FACTOR = 0.7;


    public DriveController(Robot robot) {
        this.robot = robot;
        moduleLeft = new DriveModule(robot, ModuleSide.LEFT);
        moduleRight = new DriveModule(robot, ModuleSide.RIGHT);

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();
    }

    //converts joystick vectors to parameters for update() method
    //called every loop cycle in TeleOp
    public void updateUsingJoysticks(Vector2d joystick1, Vector2d joystick2) {
        update(joystick1, -joystick2.getX() * ROBOT_ROTATION_SCALE_FACTOR);
    }

    //should be called every loop cycle when driving (auto or TeleOp)
    //note: positive rotationMagnitude is CCW rotation
    public void update(Vector2d translationVector, double rotationMagnitude) {
        moduleLeft.updateTarget(translationVector, rotationMagnitude);
        moduleRight.updateTarget(translationVector, rotationMagnitude);
    }


    //Debug methods to only update one module
    public void updateLeft(Vector2d joystick1, Vector2d joystick2) {
        moduleLeft.updateTarget(joystick1, -joystick2.getX() * ROBOT_ROTATION_SCALE_FACTOR);
    }


    public void updateRight(Vector2d joystick1, Vector2d joystick2) {
        moduleRight.updateTarget(joystick1, -joystick2.getX() * ROBOT_ROTATION_SCALE_FACTOR);
    }

    //AUTONOMOUS METHODS
    //do NOT call in a loop

    //speed should be scalar from 0 to 1
    public void drive(Vector2d direction, double cmDistance, double speed, LinearOpMode linearOpMode, double timeoutms) {
        //turns modules to correct positions for straight driving
        //rotateModules()
        resetDistanceTraveled();
        double startTime = System.currentTimeMillis();

        while (getDistanceTraveled() < cmDistance && linearOpMode.opModeIsActive() ) {
            //slows down drive power in certain range
            if (cmDistance - getDistanceTraveled() < START_DRIVE_SLOWDOWN_AT_CM) {
                speed = RobotUtil.scaleVal(cmDistance - getDistanceTraveled(), 0, START_DRIVE_SLOWDOWN_AT_CM, 0.1, 1);
            }
            updateTracking();
            update(direction.normalize(speed), 0);

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();
            if(System.currentTimeMillis()-startTime>timeoutms){
                break;
            }
        }
        update(Vector2d.ZERO, 0);
    }
    public void drive(Vector2d direction, double cmDistance, double speed, LinearOpMode linearOpMode) {
        //turns modules to correct positions for straight driving
        //rotateModules()
        resetDistanceTraveled();
        while (getDistanceTraveled() < cmDistance && linearOpMode.opModeIsActive()) {
            //slows down drive power in certain range
            if (cmDistance - getDistanceTraveled() < START_DRIVE_SLOWDOWN_AT_CM) {
                speed = RobotUtil.scaleVal(cmDistance - getDistanceTraveled(), 0, START_DRIVE_SLOWDOWN_AT_CM, 0.1, 1);
            }
            updateTracking();
            update(direction.normalize(speed), 0);

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.update();
        }
        update(Vector2d.ZERO, 0);
    }

    public void rotateRobot(Angle targetAngle, LinearOpMode linearOpMode) {
        //rotateModules
        int iterations = 0;

        boolean isNegativeRotation = robot.getRobotHeading().directionTo(targetAngle) == Angle.Direction.CLOCKWISE;

        double absHeadingDiff = robot.getRobotHeading().getDifference(targetAngle);
        while (absHeadingDiff > ALLOWED_MODULE_ROT_ERROR && linearOpMode.opModeIsActive() && iterations < MAX_ITERATIONS_ROBOT_ROTATE) {
            absHeadingDiff = robot.getRobotHeading().getDifference(targetAngle);
            double rotMag = RobotUtil.scaleVal(absHeadingDiff, 0, 45, .1, .5); //was max power 1

            if (robot.getRobotHeading().directionTo(targetAngle) == Angle.Direction.CLOCKWISE) {
                update(Vector2d.ZERO, -rotMag);
                if (!isNegativeRotation) iterations++;
            } else {
                update(Vector2d.ZERO, rotMag);
                if (isNegativeRotation) iterations++;
            }
            linearOpMode.telemetry.addData("Rotating ROBOT", "");
            linearOpMode.telemetry.update();
        }
        update(Vector2d.ZERO, 0);
    }

    //both modules must be within allowed error for method to return
    public void rotateModules(Vector2d direction,  LinearOpMode linearOpMode) {
        //TODO: check if this will work with reversed modules
        double moduleLeftDifference, moduleRightDifference;
        do {
            moduleLeftDifference = moduleLeft.getCurrentOrientation().getDifference(direction.getAngleAngle());
            moduleRightDifference = moduleRight.getCurrentOrientation().getDifference(direction.getAngleAngle());
            moduleLeft.rotateModule(direction);
            moduleRight.rotateModule(direction);

            linearOpMode.telemetry.addData("Rotating MODULES", "");
            linearOpMode.telemetry.update();
        } while ((moduleLeftDifference > ALLOWED_MODULE_ROT_ERROR || moduleRightDifference > ALLOWED_MODULE_ROT_ERROR) && linearOpMode.opModeIsActive());
        update(Vector2d.ZERO, 0);
    }



    //TRACKING METHODS
    //methods for path length tracking in autonomous (only useful for driving in straight lines)

    public void resetDistanceTraveled() {
        previousRobotDistanceTraveled = robotDistanceTraveled;
        robotDistanceTraveled = 0;

        moduleRight.resetDistanceTraveled();
        moduleLeft.resetDistanceTraveled();
    }

    public void updateTracking() {
        moduleRight.updateTracking();
        moduleLeft.updateTracking();

        double moduleLeftChange = moduleLeft.getDistanceTraveled() - moduleLeftLastDistance;
        double moduleRightChange = moduleRight.getDistanceTraveled() - moduleRightLastDistance;
        robotDistanceTraveled += (moduleLeftChange + moduleRightChange) / 2;

        moduleLeftLastDistance = moduleLeft.getDistanceTraveled();
        moduleRightLastDistance = moduleRight.getDistanceTraveled();
    }

    //note: returns ABSOLUTE VALUE
    public double getDistanceTraveled() {
        return Math.abs(robotDistanceTraveled);
    }




    public void resetEncoders() {
        moduleRight.resetEncoders();
        moduleLeft.resetEncoders();
    }

}