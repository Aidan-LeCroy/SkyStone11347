//Package
package org.firstinspires.ftc.teamcode;
//Imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveBase {
    DcMotor topL;
    DcMotor topR;
    DcMotor bottomL;
    DcMotor bottomR;
    //create array to allow for easy methods on all motors

    // initialize motors


    //motor TL with encoder, top gear left side

    //motor BL with encoder, bottom gear left side

    //motor TR with encoder, top gear right side

    //motor BR with encoder, bottom gear right side


public DriveBase() {


}
    public void init(){
        topL=hardwareMap.dcMotor.get("topL");
        topR=hardwareMap.dcMotor.get("topR");
        bottomL=hardwareMap.dcMotor.get("bottomL");
        bottomR=hardwareMap.dcMotor.get("bottomR");
        DcMotor[] motors={topL,topR,bottomL,bottomR};
        for(DcMotor i:motors){
        i=
    }


}

}