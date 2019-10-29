//Package
package org.firstinspires.ftc.teamcode;
//Imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//@TeleOp(name="TeleOp: rev1",group="DiffCore:")
public class DiffTeleOp extends DiffCore {
//USELESS RIGHT NOW, STARTING FROM DIFFCORE
    DiffCore robot=new DiffCore();
public DiffTeleOp()  {
    robot.slowMode();
    robot.resetEncoders();
    super.init();
    super.loop();


    }

}