package org.firstinspires.ftc.teamcode.TestDebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CircuitRunners.AS5600Driver;

public class AS5600DriverTesting extends LinearOpMode {


    private AS5600Driver as5600;

    @Override
    public void runOpMode() throws InterruptedException{

        as5600 = hardwareMap.get(AS5600Driver.class, "MagI2C");

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Number of Perm R/W", as5600.getNumOfPermWritesRaw());
        }

    }
}
