package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.commands;

import org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems.LiftSubsystem;

public class Move4BCommand implements Command{

    public enum v4BPos {
        OUT(.6),
        IN(0);

        public final double position;

        v4BPos(double position){
            this.position = position;
        }
    }
    
    LiftSubsystem lift;
    v4BPos pos;

    public Move4BCommand(LiftSubsystem lift, v4BPos pos){
        this.lift = lift;
        this.pos = pos;
    }

    @Override
    public void initialize() {
        lift.initialize();
    }

    @Override
    public void execute(){
        lift.set4BPos(pos.position);
    }

    @Override
    public void end() {
        if (isFinished()) lift.stop();
    }

    @Override
    public boolean isFinished() {
        return lift.get4BPos() == pos.position;
    }

}
