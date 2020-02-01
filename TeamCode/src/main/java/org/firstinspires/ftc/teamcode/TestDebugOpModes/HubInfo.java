package org.firstinspires.ftc.teamcode.TestDebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.CircuitRunners.BulkDataManager;
import org.openftc.revextensions2.ExpansionHubEx;

@TeleOp(name = "Expansion Hub Info", group = "Test")
public class HubInfo extends LinearOpMode {

    BulkDataManager bulkDataManager;
    ExpansionHubEx hub5;
    ExpansionHubEx hub7;


    @Override
    public void runOpMode(){
        hub5 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 5");
        hub7 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 7");

        bulkDataManager = new BulkDataManager(
                hub5,
                hub7
        );

        String hub5Info = hub5.getConnectionInfo();
        String hub7Info = hub7.getConnectionInfo();

        boolean h5m0 = false;
        boolean h5m1 = false;
        boolean h5m2 = false;
        boolean h5m3 = false;
        boolean h7m0 = false;
        boolean h7m1 = false;
        boolean h7m2 = false;
        boolean h7m3 = false;


        ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime updateRate = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        waitForStart();

        loopTimer.reset();
        updateRate.reset();



        while (opModeIsActive()){
            if(updateRate.milliseconds() >= 250){
                updateMotors();
                updateRate.reset();
            }

            telemetry.addLine("Expansion Powers");
            telemetry.addData("Hub 5 Draw", "%.2f Amps", bulkDataManager.hubDraw(ExpansionHubEx.CurrentDrawUnits.AMPS, 5));
            telemetry.addData("Hub 7 Draw", "%.2f Amps", bulkDataManager.hubDraw(ExpansionHubEx.CurrentDrawUnits.AMPS, 7));
            telemetry.addLine("Hub Temperatures");
            telemetry.addData("Hub 5 Over Temp", hub5.isModuleOverTemp());
            telemetry.addData("Hub 7 Over Temp", hub7.isModuleOverTemp());
            telemetry.addData("Hub 5 Motor H-Bridge Over Temp","%b , %b , %b , %b", h5m0, h5m1, h5m2, h5m3);
            telemetry.addData("Hub 7 Motor H-Bridge Over Temp", "%b , %b , %b , %b", h7m0, h7m1, h7m2, h7m3);
            telemetry.addLine("Hub Ports");
            telemetry.addData("Hub 5 5v Rail", "%.2f Volts", hub5.read5vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));
            telemetry.addData("Hub 7 5v Rail", "%.2f Volts", hub7.read5vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));
            telemetry.addData("Hub 5 GPIO Draw", "%.2f Amps", hub5.getGpioBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.addData("Hub 7 GPIO Draw", "%.2f Amps", hub7.getGpioBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.addData("Hub 5 I2C Draw", "%.2f Amps", hub5.getI2cBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.addData("Hub 7 I2C Draw", "%.2f Amps", hub7.getI2cBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.addData("Loop Time Benchmark", "%.2f ms", loopTimer.milliseconds());
            loopTimer.reset();
            telemetry.update();
        }
    }

    private void updateMotors(){
        boolean h5m0 = hub5.isMotorBridgeOverTemp(0);
        boolean h5m1 = hub5.isMotorBridgeOverTemp(1);
        boolean h5m2 = hub5.isMotorBridgeOverTemp(2);
        boolean h5m3 = hub5.isMotorBridgeOverTemp(3);
        boolean h7m0 = hub7.isMotorBridgeOverTemp(0);
        boolean h7m1 = hub7.isMotorBridgeOverTemp(1);
        boolean h7m2 = hub7.isMotorBridgeOverTemp(2);
        boolean h7m3 = hub7.isMotorBridgeOverTemp(3);
    }

    private String boolStr(boolean value){
        return value ? "Yes" : "No";
    }






}
