package org.firstinspires.ftc.teamcode.CircuitRunners;

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@I2cDeviceType
@DeviceProperties(
        name = "AS5600 Magnetic Rotary Sensor",
        description = "A magnetic sensor that tracks heading.",
        xmlTag = "AS5600",
        compatibleControlSystems = ControlSystem.REV_HUB
)
public class AS5600Driver extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    //I2c address
    public static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x36);

    //All user-intractable registers
    /*
    Some registers have two bytes
    They have HI and LOW parts to them. The number is spread across both
    i.e. 0000 0000 0000 0000 = HI{0000 0000} LOW{0000 0000}
     */
    public enum Register {

        //Configuration Registers
        FIRST(0),
        ZMCO(0x00),
        ZPOS_HI(0x01),
        ZPOS_LOW(0x02),
        MPOS_HI(0x03),
        MPOS_LOW(0x04),
        MANG_HI(0x05),
        MANG_LOW(0x06),
        CONF_HI(0x07),
        CONF_LOW(0x08),

        //Output Registers
        RAW_ANGLE_HI(0x0c),
        RAW_ANGLE_LOW(0x0d),
        ANGLE_HI(0x0E),
        ANGLE_LOW(0x0F),

        //Status Registers
        STATUS(0x0b),
        AGC(0x1a),
        MAGNITUDE_HI(0x1b),
        MAGNITUDE_LOW(0x1c),

        //Burn Commands
        BURN(0xff),

        //Read Window things
        LAST(BURN.bVal);

        public int bVal;

        Register(int bVal){
            this.bVal = bVal;
        }


    }

    protected void setOptimalReadWindow(){
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected synchronized byte read8(final Register reg){
        return deviceClient.read8(reg.bVal);
    }

    protected synchronized void write8(final Register reg, final int data){
        this.deviceClient.write8(reg.bVal, data);
    }

    //Writes two bytes of data starting at the given register
    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    //Reads two bytes of data starting at the given register
    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }
    protected void waitForWriteCompletions()
    {
        // We use ATOMIC for legacy reasons, but now that we have WRITTEN, that might
        // be a better choice.
        this.deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);
    }


    public short getNumOfPermWritesRaw(){
        return readShort(Register.ZMCO);
    }

    public void angleProgram(Telemetry telemetry){
        telemetry.addLine("Put Sensor in start position.");
        short raw_angle = readShort(Register.RAW_ANGLE_HI);
        writeShort(Register.ZPOS_HI, raw_angle);

    }

    @Override
    public Manufacturer getManufacturer(){
        return Manufacturer.Unknown;
    }

    @Override
    public synchronized boolean doInitialize(){
        return true;
    }

    @Override
    public String getDeviceName(){
        return "AS5600 Magnetic Rotary Sensor";
    }

    //Constructor to initialize device
    public AS5600Driver(I2cDeviceSynch deviceClient){
        super(deviceClient, true);

        setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); //Deals with the USB getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

}
