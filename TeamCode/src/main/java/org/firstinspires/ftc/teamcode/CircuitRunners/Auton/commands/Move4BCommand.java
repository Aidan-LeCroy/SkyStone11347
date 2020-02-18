package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.commands;

import org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems.LiftSubsystem;

public class Move4BCommand implements Command{

    public enum v4BPos {
        OUT(.85),
        IN(.03);

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
    public void execute(){
        lift.set4BPos(pos.position);
    }
}
