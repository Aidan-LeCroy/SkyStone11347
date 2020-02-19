package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.commands;

import org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems.LiftSubsystem;

/**
 * Basically, this class moves the lift to a given encoder position.
 * While it can go to down, it's not the best idea
 * (It is a blocking command)
 */
public class LiftToEncoderPosCommand implements Command {

    LiftSubsystem lift;
    double encoderPos;

    public LiftToEncoderPosCommand(LiftSubsystem lift, double encoderPos){
        this.lift = lift;
        this.encoderPos = encoderPos;
    }

    @Override
    public void initialize() {
        lift.initialize();
    }

    @Override
    public void execute(){
        lift.goTo(encoderPos);
    }

    @Override
    public void end() {
        lift.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(lift.currentPos() - encoderPos) <= lift.getTolerance();
    }
}
