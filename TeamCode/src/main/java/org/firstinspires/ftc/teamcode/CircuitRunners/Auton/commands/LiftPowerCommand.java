package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.commands;

import org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems.LiftSubsystem;

public class LiftPowerCommand implements Command {

    public enum LiftPower {
        UP(0.8),
        DOWN(-0.1), //Don't know about this one. There has to be some power though
        STOP(0);

        public final double power;

        LiftPower(double power){
            this.power = power;
        }
    }

    private LiftSubsystem lift;
    private LiftPower power;

    public LiftPowerCommand(LiftSubsystem lift, LiftPower power){
        this.lift = lift;
        this.power = power;
    }

    /* I'm going to be honest I literally have no idea how to do this well for downward movement
    The dilema I'm seeing is that, while the lift can fall under it's own power, we don't know
    exactly how fast it is doing that. My plan is to apply a very small reverse power to hopefully
    keep the tension on the string and maybe pull it down a little faster. Unfortunately, stopping
    at the bottom will probably have to be a thing done by the program itself, as this command is
    intended to be non-blocking. Keep in mind that the motors are in BRAKE (or normally are)
     */


    @Override
    public void execute(){
        lift.setPower(power.power);
    }
}
