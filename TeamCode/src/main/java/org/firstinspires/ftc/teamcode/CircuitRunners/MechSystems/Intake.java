package org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.CircuitRunners.Robot;

public class Intake {



    Gamepad gamepad2;
    Robot robot;

    private DcMotor intake_left, intake_right;

    private final double INTAKE_IN_POWER = -1;

    private final double INTAKE_OUT_POWER = 1;



    public Intake(Robot robot){
        this.robot = robot;
        this.gamepad2 = robot.gamepad2;

        intake_left = robot.findMotor("leftIntake");
        intake_right = robot.findMotor("rightIntake");

        intake_right.setDirection(DcMotor.Direction.REVERSE);


        stop();

    }

    public void update(){
        if (gamepad2.left_trigger > 0.1) {
            in();
        } else if (gamepad2.right_trigger > 0.1) {
            out();
        } else {
            stop();
        }
    }

    public void in(){
        set(INTAKE_IN_POWER);
    }

    public void out(){
        set(INTAKE_OUT_POWER);
    }

    public void stop(){
        set(0);
    }


    public void set(double power){
        intake_left.setPower(power);
        intake_right.setPower(power);
    }
}
