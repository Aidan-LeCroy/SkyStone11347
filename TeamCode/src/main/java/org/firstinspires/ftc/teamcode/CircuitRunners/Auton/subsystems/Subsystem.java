package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems;


/**
 * This interface specifies classes that are meant to be used as subsystems for the robot
 */
public interface Subsystem {

    /**
     * Should be triggered on the initiation period.
     */
    void initialize();

    /**
     * Should be triggered on the stop
     */
    void stop();

}
