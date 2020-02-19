package org.firstinspires.ftc.teamcode.CircuitRunners.Auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CircuitRunners.Auton.subsystems.*;
import org.firstinspires.ftc.teamcode.CircuitRunners.Auton.commands.*;

public class AutoRed extends LinearOpMode {


    IntakeSubsystem intake = new IntakeSubsystem(this);
    LiftSubsystem lift = new LiftSubsystem(this);
    VisionSubsystem vision = new VisionSubsystem(this);


    private static int skystonePos = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        addLog("Subsystems Initializing...");
        telemetry.update();

        intake.initialize();
        lift.initialize();
        vision.initialize();

        addLog("Initialized...");
        addLog("Starting Vision...");
        telemetry.update();

        vision.startStreaming();
        addLog("Started...");
        while(!isStarted() && !isStopRequested()){
            vision.addTelemetry();
            telemetry.update();
            //The use of the vision subsystem is special in that it's not used through a command
            //(If you make a command for vision, there is something wrong with you)
        }
        vision.stopStreaming();
        skystonePos = vision.getSkystonePos();



        //Move intake out after sizing
        new IntakeCommand(intake, IntakeSubsystem.Direction.IN).execute();
        sleep(500);
        new IntakeCommand(intake, IntakeSubsystem.Direction.STOP).execute();




        //AT THE END
        intake.stop();
        lift.stop();
        vision.stop(); //<- Just for continuity

    }

    private void addLog(String data) {
        telemetry.log().add(data);
    }
}
