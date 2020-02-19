package org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.CircuitRunners.Robot;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

@Config
public class Intake {



    Gamepad gamepad2;
    Robot robot;

    private ExpansionHubMotor intake_left, intake_right;

    public static double INTAKE_IN_POWER = -.5;

    public static double INTAKE_OUT_POWER = .5;



    public Intake(Robot robot){
        this.gamepad2 = robot.gamepad2;

        intake_left = robot.findMotor("leftIntake");
        intake_right = robot.findMotor("rightIntake");

        intake_right.setDirection(DcMotor.Direction.REVERSE);


        stop();
    }

    public Intake(LinearOpMode opMode){
        this.gamepad2 = opMode.gamepad2;

        intake_left = opMode.hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
        intake_right = opMode.hardwareMap.get(ExpansionHubMotor.class, "rightIntake");

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

    public double getLeftPower(){
        return intake_left.getPower();
    }

    public double getLeftDraw(){
        return intake_left.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
    }

    public double getRightDraw(){
        return intake_right.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
    }

    public double getRightPower(){
        return intake_right.getPower();
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

    private double getLowestCurrent(){
        return Math.min(intake_left.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS), intake_right.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
    }

    public void set(double power){
        intake_left.setPower(power * 0.9);
        intake_right.setPower(power);
    }
}
