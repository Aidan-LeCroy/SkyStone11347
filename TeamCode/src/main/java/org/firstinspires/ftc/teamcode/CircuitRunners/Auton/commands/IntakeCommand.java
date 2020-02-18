package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.commands;


import org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems.IntakeSubsystem;

public class IntakeCommand implements Command{

    IntakeSubsystem intake;
    IntakeSubsystem.Direction direction;

    public IntakeCommand(IntakeSubsystem intake, IntakeSubsystem.Direction direction){
        this.intake = intake;
        this.direction = direction;
    }




    @Override
    public void execute(){
        intake.setDirection(direction);
    }


}
