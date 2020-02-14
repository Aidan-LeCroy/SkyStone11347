package org.firstinspires.ftc.teamcode.CircuitRunners;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;



/**
 * Class that holds the bulk data information and methods
 * for using bulk data reads.
 *
 * Encompasses both hubs and their respective bulk data objects
 */
public class BulkDataManager {

    //Expansion Hub 5
    private ExpansionHubEx expansionHubEx5;
    //Expansion Hub 2
    private ExpansionHubEx expansionHubEx7;

    //Bulk data objects for both hubs
    private RevBulkData revBulkData5;
    private RevBulkData revBulkData7;

    public BulkDataManager(ExpansionHubEx expansionHubEx5,
                           ExpansionHubEx expansionHubEx7
                           ){
        this.expansionHubEx5 = expansionHubEx5;
        this.expansionHubEx7 = expansionHubEx7;

        revBulkData5 = expansionHubEx5.getBulkInputData();
        revBulkData7 = expansionHubEx7.getBulkInputData();
    }


    //Refreshes the bulk data
    public void update(){
        revBulkData5 = expansionHubEx5.getBulkInputData();
        revBulkData7 = expansionHubEx7.getBulkInputData();
    }


    //Returns encoder for hub 5
    public double getEncoder(DcMotor motor, int hubNum){
        switch (hubNum){
            case 5: return revBulkData5.getMotorCurrentPosition(motor);
            case 7: return revBulkData7.getMotorCurrentPosition(motor);
            default: return 0;
        }
    }

    //Returns encoder velocity for a motor on a hub
    public double getEncoderVelo(DcMotor motor, int hubNum){
        switch (hubNum) {
            case 5: return revBulkData5.getMotorVelocity(motor);
            case 7: return revBulkData7.getMotorVelocity(motor);
            default: return 0;
        }
    }

    //Get an analog pin output
    public double getAnalog(AnalogInput sensor, int hubNum){
        switch(hubNum){
            case 5: return revBulkData5.getAnalogInputValue(sensor);
            case 7: return revBulkData7.getAnalogInputValue(sensor);
            default: return 0;
        }
    }

    //Returns a digital state
    public boolean getDigital(DigitalChannel channel, int hubNum){
        switch (hubNum){
            case 5: return revBulkData5.getDigitalInputState(channel);
            case 7: return revBulkData7.getDigitalInputState(channel);
            //default is true because switches are inverted anyway so it's less likely to do damage
            default: return true;
        }
    }

    //Didn't add the isBusy() one because it's useless here

    //Next are Rev Extensions 2 methods that are cool and need a place

    //Set i2c speed on both hubs, use the re2 enum
    public void seti2cSpeeds(ExpansionHubEx.I2cBusSpeed speed){
        expansionHubEx5.setAllI2cBusSpeeds(speed);
        expansionHubEx7.setAllI2cBusSpeeds(speed);
    }

    //Total module draw
    public double hubDraw(ExpansionHubEx.CurrentDrawUnits unit, int hubNum){
        switch (hubNum){
            case 5: return expansionHubEx5.getTotalModuleCurrentDraw(unit);
            case 7: return expansionHubEx7.getTotalModuleCurrentDraw(unit);
            default: return 0;
        }
    }

    //Returns whether the hub is over temp
    public boolean hubOverTemp(int hubNum){
        switch (hubNum){
            case 5: return expansionHubEx5.isModuleOverTemp();
            case 7: return expansionHubEx7.isModuleOverTemp();
            default: return false;
        }
    }

    //Returns the hub temperature (PROBABLY BROKEN) in F
    public double hubTemp(int hubNum){
        switch (hubNum){
            case 5: return expansionHubEx5.getInternalTemperature(ExpansionHubEx.TemperatureUnits.FAHRENHEIT);
            case 7: return expansionHubEx7.getInternalTemperature(ExpansionHubEx.TemperatureUnits.FAHRENHEIT);
            default: return 0;
        }
    }

    //Hub current in amps
    public double hubCurrent(int hubNum){
        switch(hubNum){
            case 5:
                return expansionHubEx5.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
            case 7:
                return expansionHubEx7.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
            default: return 0;
        }
    }

    //Return bulk data object for hub 5
    public RevBulkData getBulkData5(){
        return revBulkData5;
    }

    //Return the bulk data object for hub 7
    public RevBulkData getRevBulkData7(){
        return revBulkData7;
    }



}
