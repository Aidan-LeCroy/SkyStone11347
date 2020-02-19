package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.commands;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems.DriveSubsystem;

public class DriveTranslationCommand implements Command {

    private DriveSubsystem drive;
    private DriveSubsystem.Direction posit;
    private Translation2d translation;
    private double power;

    public DriveTranslationCommand(DriveSubsystem drivebase) {
        this(drivebase, DriveSubsystem.Direction.STOP, new Translation2d(), 1);
    }

    public DriveTranslationCommand(DriveSubsystem drivebase,
                                   DriveSubsystem.Direction direction,
                                   Translation2d position,
                                   double power) {
        drive = drivebase;
        posit = direction;
        translation = position;
        this.power = power;
    }

    @Override
    public void initialize() {
        drive.initialize();
    }

    @Override
    public void execute() {
        try {
            drive.driveInDirection(posit, power);
        } catch (Exception e) {
            Log.println(1,"Warning", e.getMessage());
        }
    }

    @Override
    public void end() {
        if (isFinished()) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return drive.getTranslation().equals(translation);
    }

}
