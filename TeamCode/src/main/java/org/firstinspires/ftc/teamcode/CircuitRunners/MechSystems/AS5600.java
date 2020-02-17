package org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.CircuitRunners.BulkDataManager;
import org.openftc.revextensions2.RevBulkData;

public class AS5600  {
    // If DIR is connected to GND a clockwise rotation will increment
    // If DIR is connected to VDD a clockwise rotation will decrement

    private static final double maxVoltage = 3.3; //eg 5v, 3.3v VDD
    private static final double minVoltage = 0.0; //eg. GND
    private AnalogInput sensor;
    private double lastKnownHeading = 0;
    private double lastKnownVoltage = 0;
    public AS5600(AnalogInput sensor){
        this.sensor = sensor;

    }

    /**
     *
     * @return  raw voltage from sensor
     */
    public double rawVoltage(){
        lastKnownVoltage = sensor.getVoltage();
        return lastKnownVoltage;
    }


    public double getMinV(){
        return minVoltage;
    }

    public double getMaxV(){
        return maxVoltage;
    }

    public Func<Double> getHeadingFunction(){
        return this::getHeading;
    }

    public Func<Double> getVoltageFunction(){
        return this::rawVoltage;
    }

    //Get the heading from 0 - 360
    public double getHeading(){
        lastKnownHeading = scaleFromVoltage(
                rawVoltage(), //value
                0.0, //lower
                360.0 //upper
        );
        return lastKnownHeading;
    }

    public double getLastKnownHeading(){
        return lastKnownHeading;
    }

    public double getLastKnownVoltage(){
        return lastKnownVoltage;
    }


    //Scale a given voltage value into a given range
    public static double scaleFromVoltage(double voltage, double min, double max){
        return Range.scale(
                voltage,
                minVoltage,
                maxVoltage,
                min,
                max);
    }


}
