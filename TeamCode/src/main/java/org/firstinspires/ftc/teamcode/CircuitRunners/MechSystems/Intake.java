package org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.CircuitRunners.Robot;

public class Intake {
    
    //Cache for motors
    private static double lastRead = 0.0;
    
    //Threshold for cache
    private static final double CACHE_THRESHOLD = 0.1;
    
    private double alterFromCache(double newPower){
        if(Math.abs(lastRead - newPower) > CACHE_THRESHOLD || newPower == 0.0){
            lastRead = newPower;
            return newPower;
        }
        else {
            return lastRead;
        }
    }   


    Gamepad gamepad2;
    Robot robot;

    DcMotor intake_left, intake_right;

    private final double INTAKE_IN_POWER = -1;

    private final double INTAKE_OUT_POWER = 1;



    public Intake(Robot robot){
        this.robot = robot;
        this.gamepad2 = robot.gamepad2;

        intake_left = robot.findMotor("leftIntake");
        intake_right = robot.findMotor("rightIntake");

        intake_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        stop();

    }

    public void update(){
        if (gamepad2.left_trigger > 0) {
            in();
        } else if (gamepad2.right_trigger > 0) {
            out();
        } else {
            stop();
        }
    }

    private void in(){
        set(INTAKE_IN_POWER);
    }

    private void out(){
        set(INTAKE_OUT_POWER);
    }

    public void stop(){
        set(0);
    }


    public void set(double power){
        double altered = alterFromCache(power);
        intake_left.setPower(altered);
        intake_right.setPower(altered);
    }
}
