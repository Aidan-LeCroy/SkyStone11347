package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.DiffCore;

@TeleOp(name="Motor Test", group = "DiffSwerve")
public class MotorTest extends OpMode {
    DiffCore robot= new DiffCore(this);
    int motor;

    @Override
    public void init() {

    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            motor = 1;
        }
        if(gamepad1.b) {
            motor = 2;
        }
        if(gamepad1.x) {
            motor = 3;
        }
        if(gamepad1.y) {
            motor = 4;
        }
        if(motor == 1) {

        }
    }

    public void stop() {
        super.stop();
    }
}
