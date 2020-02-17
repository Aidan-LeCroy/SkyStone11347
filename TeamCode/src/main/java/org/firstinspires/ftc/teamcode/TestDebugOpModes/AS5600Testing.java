package org.firstinspires.ftc.teamcode.TestDebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems.AS5600;
//import org.firstinspires.ftc.teamcode.CircuitRunners.BulkDataManager;
import org.openftc.revextensions2.ExpansionHubEx;

@TeleOp(name="Magnetic Sensor Testing",group="Test")
public class AS5600Testing extends LinearOpMode {

    AS5600 sensorLeft;
    AS5600 sensorRight;
//    BulkDataManager bulkDataManager;
    Telemetry.Item magSensorLeft;
    Telemetry.Item magSensorRight;
    Telemetry.Item magSensorLeftRaw;
    Telemetry.Item magSensorRightRaw;

    @Override
    public void runOpMode() throws InterruptedException {



//        bulkDataManager = new BulkDataManager(
//                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 5"),
//                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 7")
//        );

        sensorLeft = new AS5600(
                hardwareMap.analogInput.get("Lmagnet")

        );
        sensorRight = new AS5600(
                hardwareMap.analogInput.get("Rmagnet")

        );


        waitForStart();
        initTelemetry();




        while(opModeIsActive()){
//            bulkDataManager.update();
            telemetry.update();

        }


    }

    private void initTelemetry(){
        telemetry.setCaptionValueSeparator(" -> ");
        telemetry.setItemSeparator("-----");
        telemetry.addLine("Magnetic Sensor Reported Headings");
        magSensorLeft = telemetry.addData("Left Module Rotation", "%.2f °", sensorLeft.getHeadingFunction());

        magSensorRight = telemetry.addData("Right Module Rotation", "%.2f °", sensorRight.getHeadingFunction());

        telemetry.addLine("Raw Analog Readings");

        magSensorLeftRaw = telemetry.addData("Left Sensor Raw Analog", "%.2f Volts", sensorLeft.getVoltageFunction());

        magSensorRightRaw = telemetry.addData("Right Sensor Raw Analog", "%.2f Volts", sensorRight.getVoltageFunction());

        telemetry.update();

    }
}
