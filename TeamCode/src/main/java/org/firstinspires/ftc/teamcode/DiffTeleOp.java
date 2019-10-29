//Package
package org.firstinspires.ftc.teamcode;
//Imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//@TeleOp(name="TeleOp: rev1",group="DiffCore:")
public class DiffTeleOp extends DiffCore {

    DiffCore robot=new DiffCore();



    public DiffTeleOp()  {
        super.init();
        super.loop();


    }

}