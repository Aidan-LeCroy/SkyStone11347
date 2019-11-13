package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DiffCore;


@TeleOp(name="TeleOp2.4",group="DiffSwerve")
public class DiffTeleOp extends OpMode {
    private DiffCore robot=new DiffCore(this);

    public void init(){
        robot.init();
    }
    public void start(){
        robot.start();
    }
    public void loop(){
        robot.loop();
    }

}