package org.firstinspires.ftc.teamcode.CircuitRunners;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.openftc.revextensions2.RevBulkData;

public class AS5600  {


    private static final double maxVoltage = 3.3; //eg 5v, 3.3v VDD
    private static final double minVoltage = 0.0; //eg. GND
    private AnalogInput sensor;
    private BulkDataManager bulkData;
    private double lastKnownHeading = 0;
    private static final int hubnum = 5;
    public AS5600(AnalogInput sensor, BulkDataManager bulkData){
        this.sensor = sensor;
        this.bulkData = bulkData;

    }

    public double rawVoltage(){
        return (bulkData.getAnalog(sensor,hubnum)*1000.0);
    }

    public double getMinV(){
        return minVoltage;
    }

    public double getMaxV(){
        return maxVoltage;
    }

    //Get the heading from 0 - 360
    public double getHeading(){
        double rawVoltage = rawVoltage();
        double scaled = scaleFromVoltage(
                rawVoltage, //value
                0.0, //lower
                360.0 //upper
        );
        lastKnownHeading = scaled;
        return scaled;
    }

    public double getLastKnownHeading(){
        return lastKnownHeading;
    }


    //Scale a given voltage value into a given range
    public double scaleFromVoltage(double voltage, double min, double max){
        return Range.scale(
                voltage,
                minVoltage,
                maxVoltage,
                min,
                max);
    }






}
