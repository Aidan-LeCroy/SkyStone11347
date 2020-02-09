package org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems;

import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.hardware.motors.MotorImplEx;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
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


    class CachedMotor {

        double lastReadPower;
        private final double THRESHOLD = 0.05;

        CachedMotor(double startingPower){
            this.lastReadPower = startingPower;
        }

        public boolean checkPower(double newPower){
            double power;
            if(Math.abs(lastReadPower - newPower) >= THRESHOLD || newPower == 0){
                lastReadPower = newPower;
                return true;
            }
            else {
                return false;
            }
        }
    }
    


    private static final double motorCPR = 28 * 4 * 3.9;
    
    
    //The positional coefficients
    private final double PCoefficient = 10;
    
    //The positional tolerance
    private final double tolerance = 1;

    private final double LIFT_HOLDING_POWER = 0.1;

    private final double LIFT_UP_POWER = 0.8;

    private final double LIFT_DOWN_POWER = -0.1;

    private final double TOP_POSITION = 0;
    private final double BOTTOM_POSITION = 330;
    
    private double liftTarget = BOTTOM_POSITION;
    
    //Not used for much by default in TeleOp, but good to have
    private PController liftController = new PController(PCoefficient);




    private final String[] liftMotorIds = {"lift_left", "lift_right"};
    private ExpansionHubMotor lift_left, lift_right;
    private CachedMotor left_cache, right_cache;
    private BulkDataManager bulkDataManager;
    //private DigitalChannel left_bottom_switch;
    //private DigitalChannel right_bottom_switch;


    private Robot robot;
    private boolean atBottom;



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




    public LiftSystem(Robot robot){
        this.robot = robot;
        lift_left = robot.findMotor(liftMotorIds[0]);
        lift_right = robot.findMotor(liftMotorIds[1]);

        left_cache = new CachedMotor(0);
        right_cache = new CachedMotor(0);
        //reverse left side
        lift_left.setDirection(ExpansionHubMotor.Direction.REVERSE);
        resetEncoders();
        setRUE();
        stoplift();


        liftController.setTolerance(tolerance);
        liftController.setSetPoint(liftTarget);
        


        bulkDataManager = robot.bulkDataManager;
    }


    //Update and set power to lift
    public void update(){
        
        //Power to be sent
        double finalPower = 0;
        
        //Gamepad controls
        boolean goUp = robot.controls.liftUp.value();
        boolean goDown = robot.controls.liftDown.value();
        boolean goBottom = robot.controls.liftBottom.value();

        
        //Check all possibilities for manual control
        if(!goBottom){
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
        }
        else {
             finalPower = liftController.calculate(BOTTOM_POSITION);
        }

        setLiftPower(finalPower);


    }


    private int getPosLeft(){
        return (int) round( bulkDataManager.getEncoder(lift_left, 7));
    }

    private int getPosRight(){
        return (int) round( bulkDataManager.getEncoder(lift_right, 7));
    }
    
    //Set a new target for the positional control
    public void setLiftTargetPos(double target){
           liftTarget = target;
           liftController.setSetPoint(target);
    }
    
    public boolean atTarget(){
        return liftController.atSetPoint();
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

        if(left_cache.checkPower(power)) {
            lift_left.setPower(power);
        }
        if(right_cache.checkPower(power)) {
            lift_right.setPower(power);
        }
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
