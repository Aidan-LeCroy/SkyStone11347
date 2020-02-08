package org.firstinspires.ftc.teamcode.CircuitRunners;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems.AS5600;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.security.Policy;


public class DriveModule {
    private Robot robot;

    //TODO: make sure these motors are not flipped on your drive
    private ExpansionHubMotor motor1; //top motor
    private ExpansionHubMotor motor2; //bottom motor
    
    private AS5600 rotarySensor; //the magnetic rotation sensor

    private final ModuleSide moduleSide;
    private final Vector2d positionVector;
    //position of module relative to robot COM (center of mass)

    //used for logic that allows robot to rotate modules as little as possible
    private boolean takingShortestPath = false;
    private boolean reversed = false;
    private boolean disabled=false;
    //important note about naming conventions below:
    // a MODULE rev is when the orientation of the module changes by 360 degrees
    // a WHEEL rev is when the wheel drives a distance equal to its circumference

    private static final double TICKS_PER_MODULE_REV = (1705.0 / 42.0) * 56;
    //TODO: if heading is wrong, change 56 to 28 on module rev+wheel rev
    private static final double DEGREES_PER_TICK = 360/TICKS_PER_MODULE_REV;

    //TODO: modify this variable to match drive gear ratio
    private static final double TICKS_PER_WHEEL_REV = TICKS_PER_MODULE_REV * ( 100.0 / 29.0 );
    //ticks per WHEEL revolution

    private static final double CM_WHEEL_DIAMETER = 9.6;
    private static final double CM_PER_WHEEL_REV = CM_WHEEL_DIAMETER * Math.PI;
    public static final double CM_PER_TICK = CM_PER_WHEEL_REV/TICKS_PER_WHEEL_REV;


    //used for scaling pivot component (see getPivotComponent() method)
    private final double ANGLE_OF_MAX_MODULE_ROTATION_POWER = 60;

    //if module is within this number of degrees from its target orientation,
    // no pivot power will be applied
    private final double ALLOWED_MODULE_ORIENTATION_ERROR = 5;

    //tuned variable
    private final double ROT_ADVANTAGE = .8;
    //max rotation power divided by max translation power (scaling factor)

    //this variable is set to 0.7 because when in RUN_USING_ENCODERS mode,
    // powers about ~0.7 are the same
    //setting to 1 may increase robot top speed, but may decrease accuracy
    private double MAX_MOTOR_POWER = 0.9;

    //unit vectors representing motors in the rotation power vs. translation power coordinate plane
    //more documentation on this coming soon
    private final Vector2d MOTOR_1_VECTOR = new Vector2d(1/Math.sqrt(2), 1/Math.sqrt(2));
    private final Vector2d MOTOR_2_VECTOR = new Vector2d(-1/Math.sqrt(2), 1/Math.sqrt(2));

    //variables used for path length distance tracking
    //this tracking is useful only for straight line paths
    //tracking robot x and y position is significantly more complicated,
    // and will likely be added in a later version
    private double distanceTraveled; //distance traveled (delta s) since initial encoder state
    private double lastMotor1Encoder;
    private double lastMotor2Encoder;
    private boolean headless;

    public DriveModule(Robot robot, ModuleSide moduleSide,boolean headless) {
        this.headless=headless;
        this.robot = robot;
        this.moduleSide = moduleSide;
        if (moduleSide == ModuleSide.RIGHT) {
            motor1 = robot.findMotor("topR");
            motor2 = robot.findMotor("bottomR");
            rotarySensor = new AS5600(
                    robot.findAnalog("Rmagnet"),
                    robot.bulkDataManager
            );
            positionVector = new Vector2d((double)18/2, 0);
            //points from robot center to right module
        } else {
            motor1 = robot.findMotor("topL");
            motor2 = robot.findMotor("bottomL");
            rotarySensor = new AS5600(
                    robot.findAnalog("Lmagnet"),
                    robot.bulkDataManager
            );
            positionVector = new Vector2d((double)-18/2, 0);
            //points from robot center to left module
        }

        lastMotor1Encoder = robot.bulkDataManager.getEncoder(motor1,5) ;
        lastMotor2Encoder = robot.bulkDataManager.getEncoder(motor2,5) ;

        //TODO: Make sure we want coast or brake
        // BRAKE: Jumpy, Decently Accurate
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void updateTarget (Vector2d transVec, double rotMag) {
        //translation vector and rotation magnitude
        //converts robot heading to the angle type used by Vector2d class
        if(headless) {
            Angle convertedRobotHeading = robot.getRobotHeading()
                    .convertAngle(Angle.AngleType.NEG_180_TO_180_CARTESIAN);

            //converts the translation vector from a robot centric to a field centric one
            transVec = transVec.rotate(convertedRobotHeading.getAngle());
        }
        //vector needed to rotate robot at the desired magnitude
        //based on positionVector of module (see definition for more info)
        Vector2d rotVec = positionVector.normalize(rotMag);
        // theoretically this should be rotated 90, not sure sure it doesn't need to be

        //combine desired robot translation and robot rotation to get goal vector for the module;//
        Vector2d targetVector = transVec.add(rotVec);

        //allows modules to reverse power instead of rotating 180 degrees
        //example: useful when going from driving forwards to driving backwards
        int directionMultiplier = 1;
        if (reversed) { //reverse direction of translation because module is reversed
            targetVector = targetVector.reflect();
            directionMultiplier = -1;
        }

        //calls method that will apply motor powers necessary to reach
        // the target vector in the best way possible, based on current position
        goToTarget(targetVector, directionMultiplier);

//        robot.telemetry.addData(moduleSide + " REVERSED: ", reversed);
//        robot.telemetry.addData(moduleSide + " Trans Vec FC: ", transVecFC);
//        robot.telemetry.addData(moduleSide + " Rot Vec: ", rotVec);
    }


    //sets motor powers for robot to best approach given target vector
    private void goToTarget(Vector2d targetVector, int directionMultiplier) {
        //how much the module needs to translate (and in which direction)
        double moveComponent = targetVector.getMagnitude() * directionMultiplier;

        //how much the module needs to pivot (change its orientation)
        double pivotComponent;
        if (targetVector.getMagnitude() != 0) {
            pivotComponent = getPivotComponent(targetVector, getCurrentOrientation());
        } else {
            //if target vector is zero (joystick is within deadband) don't pivot modules
            pivotComponent = 0;
        }

        //vector in an (invented) coordinate system
        //represents desired (relative) module translation/module rotation
        Vector2d powerVector = new Vector2d(moveComponent, pivotComponent);
        //order very important here
        setMotorPowers(powerVector);

    }


    //returns a scalar corresponding to how much power the module needs to apply to rotating
    //this is necessary because of the differential nature of a diff swerve drive
    private double getPivotComponent(Vector2d targetVector, Angle currentAngle) {
        Angle targetAngle = targetVector.getAngleAngle();
        double angleDiff = targetAngle.getDifference(currentAngle);
        //number from 0 to 180 (always positive)

        //allows module to rotate to the opposite position of (180 degrees away from) its target
        //if this is the fastest path,
        // we need to indicate that the direction of translation should be reversed
        if (Math.abs(angleDiff) > 90) {
            if (!takingShortestPath) {
                reversed = !reversed; //reverse translation direction bc module is newly reversed
            }
            takingShortestPath = true;
        } else {
            takingShortestPath = false;
        }

//        robot.telemetry.addData(moduleSide + " Angle diff (abs. value): ", angleDiff);
        Angle.Direction direction = currentAngle.directionTo(targetAngle);

        //CCW is negative for heading system
        if (angleDiff < ALLOWED_MODULE_ORIENTATION_ERROR) {
            //don't rotate module if it's currently within x degrees of its target orientation
            //avoids constant twitching of modules
            return 0;
        } else if (angleDiff > ANGLE_OF_MAX_MODULE_ROTATION_POWER) {
            //rotation power is maxed out if the difference is more than this angle
            if (direction == Angle.Direction.CLOCKWISE) return ROT_ADVANTAGE;
            else return -1 * ROT_ADVANTAGE;
        } else {
            //scale module rotation power based on set constants
            if (direction == Angle.Direction.CLOCKWISE)
                return angleDiff / ANGLE_OF_MAX_MODULE_ROTATION_POWER * ROT_ADVANTAGE;
            else return -1 * angleDiff / ANGLE_OF_MAX_MODULE_ROTATION_POWER * ROT_ADVANTAGE;
        }
    }


    //takes vector in power vector coordinate system
    // x component is relative translation power
    // y component is relative MODULE rotation power
    //sets motors to appropriate powers
    private void setMotorPowers(Vector2d powerVector) {

        //this is one way to convert desired ratio of module translation
        //and module rotation to motor powers
        //vectors are not strictly necessary for this, but made it easier to visualize
        //more documentation on this visualization method coming soon
        Vector2d motor1Unscaled = powerVector.projection(MOTOR_1_VECTOR);
        Vector2d motor2Unscaled = powerVector.projection(MOTOR_2_VECTOR);

        //makes sure no vector magnitudes exceed the maximum motor power
        Vector2d[] motorPowersScaled =
                Vector2d.batchNormalize(MAX_MOTOR_POWER, motor1Unscaled, motor2Unscaled);
        double motor1power = motorPowersScaled[0].getMagnitude();
        double motor2power = motorPowersScaled[1].getMagnitude();

        //this is to add sign to magnitude, which returns an absolute value
        if (motorPowersScaled[0].getAngle() != MOTOR_1_VECTOR.getAngle()) {
            motor1power *= -1;
        }
        if (motorPowersScaled[1].getAngle() != MOTOR_2_VECTOR.getAngle()) {
            motor2power *= -1;
        }

//        robot.telemetry.addData(moduleSide + " Motor 1 Power: ", motor1power);
//        robot.telemetry.addData(moduleSide + " Motor 2 Power: ", motor2power);
        motor1.setPower(motor1power);
        motor2.setPower(motor2power);
    }


    //for pure module rotation (usually used for precise driving in auto)
    public void rotateModule (Vector2d direction) {
        //pass 0 as moveComponent
        Vector2d powerVector = new Vector2d
                (0, getPivotComponent(direction, getCurrentOrientation()));
        //order important here
        setMotorPowers(powerVector);
//        robot.telemetry.addData(moduleSide + " Power Vector: ", powerVector);
//        robot.telemetry.addData(moduleSide + " Current orientation: ",
//        getCurrentOrientation().getAngle());
    }

    //does not need to be called at the start of every program
    //separate opmode called ResetEncoders calls this method
    public void resetEncoders () {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //returns module orientation relative to ROBOT (not field) in degrees and
    // NEG_180_TO_180_HEADING type
    public Angle getCurrentOrientation() {
//        robot.telemetry.addData(moduleSide + "Motor 1 Encoder", motor1.getCurrentPosition());
//        robot.telemetry.addData(moduleSide + "Motor 2 Encoder", motor2.getCurrentPosition());
        double rawAngle = rotarySensor.getHeading();
        //motor2-motor1 makes ccw positive (?)
        return new Angle(rawAngle, Angle.AngleType.ZERO_TO_360_HEADING);
    }


    //TRACKING METHODS
    //used for straight line distance tracking

    public void updateTracking () {
        /*important to set these to a variable so getCurrentPosition()
         is not called multiple times in single cycle */
        double currentMotor1Encoder = robot.bulkDataManager.getEncoder(motor1,5);
        double currentMotor2Encoder = robot.bulkDataManager.getEncoder(motor2,5);

        double motor1Change = currentMotor1Encoder - lastMotor1Encoder;
        double motor2Change = currentMotor2Encoder - lastMotor2Encoder;

        //if module is reversed, subtract distance traveled instead of adding
        //module is driving in the opposite direction that the encoders "think" it is
        if (reversed) {
            distanceTraveled -= (motor1Change - motor2Change)/2.0 * CM_PER_TICK;
        } else {
            distanceTraveled += (motor1Change - motor2Change)/2.0 * CM_PER_TICK;
        }

        lastMotor1Encoder = currentMotor1Encoder;
        lastMotor2Encoder = currentMotor2Encoder;

//        robot.telemetry.addData(moduleSide + "Motor 1 Encoder", motor1.getCurrentPosition());
//        robot.telemetry.addData(moduleSide + "Motor 2 Encoder", motor2.getCurrentPosition());
        robot.telemetry.update();
    }

    public void resetDistanceTraveled () {
        distanceTraveled = 0;
    }

    public void directDrive(double power){
        motor1.setPower(power);
        motor2.setPower(-power);
    }
    //returns distance (in cm) traveled since distance was last reset
    public double getDistanceTraveled () {
        return distanceTraveled;
    }
    public void moduleRotate(double power){
        motor1.setPower(power);
        motor2.setPower(power);
    }
    public void disable(){
        disabled=true;
    }
    public void applyPowersToReachZero(){
        double currAngle=getCurrentOrientation().convertAngle(Angle.AngleType.NEG_180_TO_180_HEADING).getAngle();
        if(currAngle>=5){
            moduleRotate(-.5);
        }
        else if(currAngle<=-5){
            moduleRotate(.5);
        }
        else {
            moduleRotate(0);
        }
    }
}
