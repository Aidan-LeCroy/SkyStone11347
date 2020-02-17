package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake implements Subsystem {

    private LinearOpMode opMode;

    private static final double IN_POWER = 0.8;
    private static final double OUT_POWER = 0.7;


    private DcMotor intake_left;
    private DcMotor intake_right;



    public Intake(LinearOpMode opMode){
        this.opMode = opMode;
    }


    @Override
    public void onInit(){

        intake_left = opMode.hardwareMap.dcMotor.get("leftIntake");
        intake_right = opMode.hardwareMap.dcMotor.get("rightIntake");

        intake_right.setDirection(DcMotor.Direction.REVERSE);

        stop();
    }

    //Makes the intake spin inwards
    public void in(){
        setPower(IN_POWER);
    }

    //Makes the intake spin outwards
    public void out(){
        setPower(OUT_POWER);
    }



    //Stops the motors
    public void stop(){
        setPower(0);
    }

    //Internal setting power
    private void setPower(double p){
        intake_left.setPower(p);
        intake_right.setPower(p);
    }

    @Override
    public void onStop(){
        stop();
    }

}
