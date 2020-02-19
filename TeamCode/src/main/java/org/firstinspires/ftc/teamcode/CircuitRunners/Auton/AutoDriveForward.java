package org.firstinspires.ftc.teamcode.CircuitRunners.Auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.CircuitRunners.Auton.commands.*;
import org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems.*;

public class AutoDriveForward extends LinearOpMode {



    private IntakeSubsystem intake = new IntakeSubsystem(this);

    @Override
    public void runOpMode() throws InterruptedException {

        intake.onInit();

        waitForStart();

        new IntakeCommand(intake, IntakeSubsystem.Direction.IN).execute();
        sleep(500);
        new IntakeCommand(intake, IntakeSubsystem.Direction.STOP).execute();

    }
}
