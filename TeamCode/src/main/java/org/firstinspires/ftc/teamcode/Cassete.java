package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

public class Cassete {
    double wheelAngle;
    double topEncoder;
    double bottomEncoder;

    public double getWheelAngle(){
        return wheelAngle;
    }
    public double getDeltaEncoder(DcMotor top,DcMotor bottom){
        return ((top.getCurrentPosition()+bottom.getCurrentPosition())/2);
    }




}