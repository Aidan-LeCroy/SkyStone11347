package org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.hardware.motors.MotorImplEx;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static java.lang.Math.round;

import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.CircuitRunners.BulkDataManager;
import org.firstinspires.ftc.teamcode.CircuitRunners.Robot;
import org.openftc.revextensions2.ExpansionHubMotor;

public class LiftSystem {

        /*
    So the mentality behind this system is the following. The measured value (pv) is always updated,
    and when there is manual control active the setpoint (sv) is also updated. When manual control stops,
    the pv is still updated but the sv stops being updated. The output of the controller is only applied
    when there is no manual controller
     */

    


    private static final double motorCPR = 28 * 4 * 3.9;
    
    //The positional tolerance
    private final double tolerance = 1;


    private final double LIFT_UP_POWER = 0.8;

    private final double LIFT_DOWN_POWER = -0.1;

    private final double TOP_POSITION = 0;
    private final double BOTTOM_POSITION = 330;
    
    private double currentPos = 0;
    
    //Not used for much by default in TeleOp, but good to have
    private PIDFController liftController = new PIDFController(new double[] {1, .2, 0, 0});




    private final String[] liftMotorIds = {"lift_left", "lift_right"};
    private ExpansionHubMotor lift_left, lift_right;
    //private DigitalChannel left_bottom_switch;
    //private DigitalChannel right_bottom_switch;




    //Get current lift position from encoders
    public final Func<Integer> liftPosition = () -> (int) Math.round((getPosLeft() + getPosRight())/2.0);

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


    private Gamepad gamepad2;


    public LiftSystem(Robot robot){
        this.gamepad2 = robot.gamepad2;

        lift_left = robot.findMotor(liftMotorIds[0]);
        lift_right = robot.findMotor(liftMotorIds[1]);

        //reverse left side
        lift_left.setDirection(ExpansionHubMotor.Direction.REVERSE);
        resetEncoders();
        setRUE();
        stoplift();
        liftController.setTolerance(tolerance);
    }

    public LiftSystem(LinearOpMode opMode){
        this.gamepad2 = opMode.gamepad2;

        lift_left = opMode.hardwareMap.get(ExpansionHubMotor.class, liftMotorIds[0]);
        lift_right = opMode.hardwareMap.get(ExpansionHubMotor.class, liftMotorIds[1]);
        //reverse left side
        lift_left.setDirection(ExpansionHubMotor.Direction.REVERSE);
        resetEncoders();
        setRUE();
        stoplift();
        liftController.setTolerance(tolerance);
        liftController.setSetPoint(0);
    }


    //Update and set power to lift
    public void update(){

        //Update the known lift position
        currentPos = liftPosition.value();
        
        //Power to be sent
        double finalPower = 0;
        
        //Gamepad controls
        boolean goUp = gamepad2.dpad_up;
        boolean goDown = gamepad2.dpad_down;

        
        //Check all possibilities for manual control
            if(goUp && !liftAtTop.value()){
                finalPower = LIFT_UP_POWER;
                //Update the setpoint
                liftController.setSetPoint(currentPos);
            }
            else if(goDown && !liftAtBottom.value()){
                finalPower = LIFT_DOWN_POWER;
                //update the setpoint
                liftController.setSetPoint(currentPos);
            }
            else {
                //This happens whenever the lift isn't being controlled
                //Basically the pv is updated here. the sv isn't, so it holds it's position
                finalPower = liftController.calculate(currentPos);
            }

        setLiftPower(finalPower);


    }


    private int getPosLeft(){
        return lift_left.getCurrentPosition();
    }

    private int getPosRight(){
        return lift_right.getCurrentPosition();
    }



    private void resetEncoders(){
        lift_left.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRUE(){
        lift_left.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        lift_right.setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setLiftPower(double power) {
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
