package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.commands;

import org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems.LiftSubsystem;

public class GrabberCommand implements Command {

    public enum GrabberPos {
        CLOSE(.5),
        OPEN(.6);

        public final double position;

        GrabberPos(double position){
            this.position = position;
        }
    }

    private LiftSubsystem lift;
    private GrabberPos pos;

    public GrabberCommand(LiftSubsystem lift, GrabberPos pos){
        this.lift = lift;
        this.pos = pos;
    }


    @Override
    public void execute(){
        lift.setGrabPos(pos.position);
    }
}
