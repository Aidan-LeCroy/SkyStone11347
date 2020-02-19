package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.commands;

public interface Command {

    void initialize();

    void execute();

    void end();

    boolean isFinished();

}
