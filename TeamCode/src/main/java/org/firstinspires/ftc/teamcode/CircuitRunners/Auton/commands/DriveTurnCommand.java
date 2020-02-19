package org.firstinspires.ftc.teamcode.CircuitRunners.Auton.commands;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems.DriveSubsystem;

public class DriveTurnCommand implements Command {

    private DriveSubsystem drive;
    private Rotation2d rot;
    private DriveSubsystem.Direction direction;
    private double power;

    public DriveTurnCommand(DriveSubsystem drivebase) {
        this(drivebase, DriveSubsystem.Direction.STOP, new Rotation2d(), 1);
    }

    public DriveTurnCommand(DriveSubsystem drivebase,
                            DriveSubsystem.Direction direction,
                            Rotation2d rotation, double power) {
        drive = drivebase;
        rot = rotation;
        this.direction = direction;
        this.power = power;
    }

    @Override
    public void initialize() {
        drive.initialize();
    }

    @Override
    public void execute() {
        try {
            drive.driveInDirection(direction, power);
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
        return drive.getRotation() == rot.getRadians();
    }

}
