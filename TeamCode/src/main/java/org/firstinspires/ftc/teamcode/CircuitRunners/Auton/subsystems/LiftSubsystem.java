package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubMotor;

public class LiftSubsystem implements Subsystem {


    PIDFController liftController = new PIDFController(new double[] {1, .2, 0, 3});


    private ExpansionHubMotor lift_left, lift_right;


    private static final double TOLERANCE  = 1;

    private LinearOpMode opMode;
    /*
    So the mentality behind this system is the following. The measured value (pv) is always updated,
    and when there is manual control active the setpoint (sv) is also updated. When manual control stops,
    the pv is still updated but the sv stops being updated. The output of the controller is only applied
    when there is no manual controller
     */

    public LiftSubsystem(LinearOpMode opMode){
        this.opMode = opMode;
    }


    @Override
    public void onInit(){

        lift_left = opMode.hardwareMap.get(ExpansionHubMotor.class, "lift_left");
        lift_right = opMode.hardwareMap.get(ExpansionHubMotor.class, "lift_right");

        lift_left.setDirection(ExpansionHubMotor.Direction.REVERSE);
        lift_left.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(ExpansionHubMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();
        setMode(ExpansionHubMotor.RunMode.RUN_USING_ENCODER);
        stoplift();

        liftController.setTolerance(TOLERANCE);
    }

    /**
     * This is a blocking method!
     * @param pos the encoder pos
     */
    public void goTo(double pos){
        liftController.setSetPoint(pos);
        while(liftController.atSetPoint() && opMode.opModeIsActive()){
            liftController.calculate(currentPos());
        }
    }

    @Override
    public void onStop(){
        stoplift();
    }

    //Returns the current position of the lift
    public double currentPos(){
        return (lift_left.getCurrentPosition() + lift_right.getCurrentPosition()) / 2;
    }

    public void setPower(double power){
        lift_left.setPower(power);
        lift_right.setPower(power);
    }

    public void stoplift(){
        setPower(0);
    }

    private void setMode(ExpansionHubMotor.RunMode mode){
        lift_left.setMode(mode);
        lift_right.setMode(mode);
    }

    private void resetEncoders(){
        setMode(ExpansionHubMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
