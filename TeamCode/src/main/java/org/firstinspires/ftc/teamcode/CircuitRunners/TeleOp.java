package org.firstinspires.ftc.teamcode.CircuitRunners;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
        (name = "Differential Swerve TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {

    private MediaPlayer mp = new MediaPlayer();
    private Robot robot;
    //deadband for joysticks
    private double DEADBAND_MAG = 0.1;
    private Vector2d DEADBAND_VEC = new Vector2d(DEADBAND_MAG, DEADBAND_MAG);
    private boolean changed = false;
    private boolean grabbing=false;
    private boolean willResetIMU = true;
    private boolean bounce1 = false;
    private boolean bounce2 = false;
    private boolean bounce3= false;
    private boolean fbDrop=false;

    public void init() {
        robot = new Robot(this, false,false);
        try {
            mp = MediaPlayer.create(FtcRobotControllerActivity.getContext(), R.raw.megalovania);
            mp.start();
        } catch (Exception e) {
            telemetry.addData("Exception Thrown! ", e);
        }
    }


    //allows driver to indicate that the IMU should be reset
    //used when starting TeleOp after auto or if program crashes in the middle of match
    //relevant because of field-centric controls
    public void init_loop() {
        if (gamepad1.y) {
            willResetIMU = true;
            telemetry.addData("tel", "RESETTING IMU ON START, PRESS X TO NOT DO THIS");
        }
        if (gamepad1.x) {
            willResetIMU = false;
        }
    }

    public void start() {
        if (willResetIMU) robot.initIMU();
        robot.set4BPos(.5);
    }

    public void loop() {
        Vector2d joystick1 = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        //LEFT joystick
        Vector2d joystick2 = new Vector2d(gamepad1.right_stick_x, 0);
        //RIGHT joystick
//        flipFoundation();


        robot.driveController.updateUsingJoysticks
                (checkDeadband(joystick1), checkDeadband(joystick2));




        if (gamepad2.left_trigger > 0) {
            robot.intake(-1);
        } else if (gamepad2.right_trigger > 0) {
            robot.intake(1);
        } else {
            robot.intake(0);
        }
        telemetry.addData("Lift Encoder:",robot.getLiftEncoders());
        robot.manLift(.5 * gamepad2.left_stick_y);
//        //uncomment for live tuning of ROT_ADVANTAGE constant
//        if (gamepad1.b) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE += 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE += 0.01;
//        }
//        if (gamepad1.x) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE -= 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE -= 0.01;
//        }
//        telemetry.addData("ROT_ADVANTAGE: ", robot.driveController.moduleLeft.ROT_ADVANTAGE);
        if(gamepad2.x&&!bounce3){
            flip4B();
            bounce3=true;
        }
        else if(!gamepad2.x){
            bounce3=false;
        }
        if(gamepad2.a&&!bounce1){
            grabToggle();
            bounce1=true;
        }
        else if(!gamepad2.a){
            bounce1=false;
        }
        if(gamepad2.y){
            robot.set4BPos(.95);
        }

        //to confirm that joysticks are operating properly
        telemetry.addData("Joystick 1", joystick1);
        telemetry.addData("Joystick 2", joystick2);

        telemetry.update();


    }

    //returns zero vector if joystick is within deadband
    private Vector2d checkDeadband(Vector2d joystick) {
        if (Math.abs(joystick.getX()) > DEADBAND_VEC.getX() ||
                Math.abs(joystick.getY()) > DEADBAND_VEC.getY()) {
            return joystick;
        }
        return Vector2d.ZERO;
    }

    private void flip4B(){
        if(!fbDrop){
            robot.set4BPos(.03);
            fbDrop=true;
        }
        else if(fbDrop){
            robot.set4BPos(.85);
            fbDrop=false;
        }
    }
    private void grabToggle(){
        if(!grabbing){
            robot.setGrabPos(.5);
            grabbing=true;
        }
        else if(grabbing){
            robot.setGrabPos(.6);
            grabbing=false;
        }
    }

}
//    private void flipFoundation(){
//        if(gamepad2.x&& !changed) {
//            foundationDown = !foundationDown;
//            if(foundationDown)
//                robot.dropFoundation();
//            else
//                robot.liftFoundation();
//            changed=true;
//        }
//        else if(!gamepad1.x)
//            changed=false;
//    }
//
