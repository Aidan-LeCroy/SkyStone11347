package org.firstinspires.ftc.teamcode.CircuitRunners;

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;


@I2cDeviceType
@DeviceProperties(name = "AS5600 Magnetic Rotary Sensor",
        description = "Magnetic Rotary Sensor from Grove",
        xmlTag = "AS5600",
        compatibleControlSystems = ControlSystem.REV_HUB)
public class AS5600 extends I2cDeviceSynchDevice<I2cDeviceSynch> {


    @Override
    public Manufacturer getManufacturer(){
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {

        return "Grove Magnetic Rotary Sensor";
    }


    public AS5600(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);


        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

}
