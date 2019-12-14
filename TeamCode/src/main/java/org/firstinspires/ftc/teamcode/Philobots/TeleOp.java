package org.firstinspires.ftc.teamcode.Philobots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.Tracking;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Diff Swerve TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    Robot robot;
//    Tracking track;
    //deadband for joysticks
    public double DEADBAND_MAG = 0.1;
    public Vector2d DEADBAND_VEC = new Vector2d(DEADBAND_MAG, DEADBAND_MAG);
    public boolean willResetIMU = false;
    private boolean changed=false;
    private boolean foundationDown=false;
    public void init() {
        robot = new Robot(this, false);
//        track=new Tracking(hardwareMap,telemetry);
//        track.initializeCamera();


    }

    //allows driver to indicate that the IMU should not be reset
    //used when starting TeleOp after auto or if program crashes in the middle of match
    //relevant because of field-centric controls
    public void init_loop() {
        if (gamepad1.y) {
            willResetIMU = true;
        }
        if (gamepad1.x){
            willResetIMU=false;
        }
    }
    public void start () { if (willResetIMU) robot.initIMU(); }


    public void loop() {
        Vector2d joystick1 = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y); //LEFT joystick
        Vector2d joystick2 = new Vector2d(gamepad1.right_stick_x, -gamepad2.right_stick_y); //RIGHT joystick
        flipFoundation();
        robot.driveController.updateUsingJoysticks(checkDeadband(joystick1), checkDeadband(joystick2));


        //uncomment for live tuning of ROT_ADVANTAGE constant
        //TODO: After tuning, make ROT_ADVANTAGE final
//        if (gamepad1.b) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE += 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE += 0.01;
//        }
//        if (gamepad1.x) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE -= 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE -= 0.01;
//        }
        telemetry.addData("ROT_ADVANTAGE: ", robot.driveController.moduleLeft.ROT_ADVANTAGE);


        //to confirm that joysticks are operating properly
        telemetry.addData("Joystick 1", joystick1);
        telemetry.addData("Joystick 2", joystick2);
//        telemetry.addData("Skystone xpos",track.getSkyStoneX());
//        telemetry.addData("Skystone ypos",track.getSkyStoneY());

        telemetry.update();
    }

    //returns zero vector if joystick is within deadband
    public Vector2d checkDeadband(Vector2d joystick) {
        if (Math.abs(joystick.getX()) > DEADBAND_VEC.getX() || Math.abs(joystick.getY()) > DEADBAND_VEC.getY()) {
            return joystick;
        }
        return Vector2d.ZERO;
    }
    private void flipFoundation(){
        if(gamepad2.x&& !changed) {
            foundationDown = !foundationDown;
            if(foundationDown)
                robot.dropFoundation();
            else
                robot.liftFoundation();
            changed=true;
        }
        else if(!gamepad1.x)
            changed=false;
    }

}
