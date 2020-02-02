package org.firstinspires.ftc.teamcode.CircuitRunners;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static java.lang.Math.round;

import org.openftc.revextensions2.ExpansionHubMotor;

enum AboveStonePos {
    LIFT_DOWN(0),
    FIRST_LEVEL(30),
    SECOND_LEVEL(60),
    THIRD_LEVEL(90),
    FOURTH_LEVEL(120),
    FIFTH_LEVEL(150),
    SIXTH_LEVEL(180),
    SEVENTH_LEVEL(210),
    EIGTH_LEVEL(240),
    NINTH_LEVEL(270),

    TENTH_lEVEL(300), //????? MAYBE
    LIFT_TOP(330);

    public final int encoderNum;

    AboveStonePos(int encoderNum){
        this.encoderNum = encoderNum;
    }


}

enum LiftControl {
    AUTOMATIC,
    MANUAL,
    NOTHING
}

public class LiftSystem {






    private final String[] liftMotorIds = {"lift_left", "lift_right"};
    private ExpansionHubMotor lift_left, lift_right;
    private BulkDataManager bulkDataManager;
    //private DigitalChannel left_bottom_switch;
    //private DigitalChannel right_bottom_switch;
    private Gamepad gamepad2;

    private AboveStonePos liftPositions = AboveStonePos.LIFT_DOWN;
    private LiftControl liftControl = LiftControl.NOTHING;

    private Robot robot;

    private boolean togglePressed = false;
    private boolean toggleHeld = false;


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

    public void update(){



    }

    private void setTarget(AboveStonePos pos){
        setTarget(pos.encoderNum);
    }

    private double getAveragePos(){
        return (getPosLeft() + getPosRight())/2;
    }

    private int getPosLeft(){
        return (int) round( bulkDataManager.getEncoder(lift_left, 7));
    }

    private int getPosRight(){
        return (int) round( bulkDataManager.getEncoder(lift_right, 7));
    }



    private void setTarget(int pos){
        lift_left.setTargetPosition(pos);
        lift_right.setTargetPosition(pos);
    }

    private void setRTP(){
        lift_left.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);
        lift_right.setMode(ExpansionHubMotor.RunMode.RUN_TO_POSITION);
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



    //Stop all motion
    public void stoplift(){
        lift_left.setPower(0);
        lift_right.setPower(0);
    }




}
