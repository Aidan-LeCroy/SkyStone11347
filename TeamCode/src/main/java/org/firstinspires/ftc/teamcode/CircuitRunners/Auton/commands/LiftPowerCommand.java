package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.commands;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems.LiftSubsystem;

public class LiftPowerCommand implements Command {

    public enum LiftPower {
        UP(0.8),
        DOWN(-0.4), //Don't know about this one. There has to be some power though
        STOP(0);

        public final double power;

        LiftPower(double power){
            this.power = power;
        }
    }

    private LiftSubsystem lift;
    private LiftPower power;

    private PIDFController liftVelocityController;

    public LiftPowerCommand(LiftSubsystem lift, LiftPower power, PIDFController controller){
        this.lift = lift;
        this.power = power;
        liftVelocityController = controller;
    }

    @Override
    public void initialize() {
        lift.initialize();
    }

    @Override
    public void execute(){
        lift.setPower(power.power);
    }

    @Override
    public void end() {
        lift.stop();
        liftVelocityController.reset();
    }

    public boolean atBottom() {
        return lift.currentPos() <= lift.getTolerance() && lift.currentPos() >= 0;
    }

    @Override
    public boolean isFinished() {
        return liftVelocityController.atSetPoint();
    }

    public void disable() {

    }
}
