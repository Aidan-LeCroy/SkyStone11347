package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem implements Subsystem {

    public enum Direction {
        IN(-0.3),
        OUT(0.3),
        STOP(0);

        public final double power;

        Direction(double power){
            this.power = power;
        }
    }

    private LinearOpMode opMode;

    private DcMotor intake_left;
    private DcMotor intake_right;

    public IntakeSubsystem(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public void initialize(){
        intake_left = opMode.hardwareMap.dcMotor.get("leftIntake");
        intake_right = opMode.hardwareMap.dcMotor.get("rightIntake");

        intake_right.setDirection(DcMotor.Direction.REVERSE);

        stop();
    }

    //Makes the intake spin inwards
    public void in(){
        setPower(Direction.IN.power);
    }

    //Makes the intake spin outwards
    public void out(){
        setPower(Direction.OUT.power);
    }

    public void setDirection(Direction direction){
        setPower(direction.power);
    }

    public double getPower() { return (intake_left.getPower() + intake_right.getPower()) / 2; }

    //Stops the motors
    @Override
    public void stop(){
        setDirection(Direction.STOP);
    }

    //Internal setting power
    private void setPower(double p){
        intake_left.setPower(p);
        intake_right.setPower(p);
    }

}
