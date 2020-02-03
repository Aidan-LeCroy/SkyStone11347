package org.firstinspires.ftc.teamcode.CircuitRunners;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static java.lang.Math.round;

import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.robotcore.external.Func;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

public class LiftSystem {

    private final double LIFT_HOLDING_POWER = 0.1;

    private final double LIFT_UP_POWER = 0.8;

    private final double LIFT_DOWN_POWER = -0.1;

    private final double TOP_POSITION = 0;
    private final double BOTTOM_POSITION = 330;




    private final String[] liftMotorIds = {"lift_left", "lift_right"};
    private ExpansionHubMotor lift_left, lift_right;
    private BulkDataManager bulkDataManager;
    //private DigitalChannel left_bottom_switch;
    //private DigitalChannel right_bottom_switch;
    private Gamepad gamepad2;

    private Robot robot;
    private boolean atBottom;

    //Control to move lift up manually
    private final Func<Boolean> upControl = () -> gamepad2.dpad_up;

    //Control to move lift down manually
    private final Func<Boolean> downControl = () -> gamepad2.dpad_down;

    //Get current lift position from encoders
    public final Func<Integer> liftPosition = () -> (getPosLeft() + getPosRight())/2;

    //Is lift at bottom?
    private final Func<Boolean> liftAtBottom = () -> {
        if(liftPosition.value() <= BOTTOM_POSITION){
            return true;
        }
        else {
            return false;
        }
    };

    //Is lift at top?
    private final Func<Boolean> liftAtTop = () -> {
        if(liftPosition.value() >= TOP_POSITION){
            return true;
        }
        else {
            return false;
        }
    };




    public LiftSystem(Robot robot){
        this.robot = robot;
        lift_left = robot.findMotor(liftMotorIds[0]);
        lift_right = robot.findMotor(liftMotorIds[1]);

        //reverse left side
        lift_left.setDirection(ExpansionHubMotor.Direction.REVERSE);
        stoplift();
        resetEncoders();


        bulkDataManager = robot.bulkDataManager;
        gamepad2 = robot.gamepad2;
    }


    //Update and set power to lift
    public void update(){

        boolean goUp = upControl.value();
        boolean goDown = downControl.value();

        double finalPower;
        if(goUp && !liftAtTop.value()){
            finalPower = LIFT_UP_POWER;
        }
        else if(goDown && !liftAtBottom.value()){
            finalPower = LIFT_DOWN_POWER;
        }
        else if(!liftAtBottom.value()){
            finalPower = LIFT_HOLDING_POWER;
        }
        else {
            finalPower = 0;
        }

        setLiftPower(finalPower);


    }


    private int getPosLeft(){
        return (int) round( bulkDataManager.getEncoder(lift_left, 7));
    }

    private int getPosRight(){
        return (int) round( bulkDataManager.getEncoder(lift_right, 7));
    }


    private void resetEncoders(){
        lift_left.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRUE(){
        lift_left.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        lift_right.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setLiftPower(double power){
        lift_left.setPower(power);
        lift_right.setPower(power);
    }

    //Is left motor over temp?
    public boolean isLeftOverTemp(){
        return lift_left.isBridgeOverTemp();
    }

    //Is right motor over temp?
    public boolean isRightOverTemp(){
        return lift_right.isBridgeOverTemp();
    }

    //Stop all motion
    public void stoplift(){
        lift_left.setPower(0);
        lift_right.setPower(0);
    }




}
