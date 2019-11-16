package org.firstinspires.ftc.teamcode.GK;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="GK-TeleOp", group="GK")
public class GKTeleOp extends GKCore {

    public GKTeleOp() {
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {
        super.init();
        telemetry.addData("Status", "Initialized");
        gamepad1.setJoystickDeadzone(.1f);
        gamepad2.setJoystickDeadzone(.1f);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        time.reset();
        telemetry.addData("Evan", " has been gay for " + (int) time.seconds() + " seconds");
        super.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        frontR.setPower(gamepad1.right_stick_y);
        backR.setPower(gamepad1.right_stick_y);
        frontL.setPower(gamepad1.left_stick_y);
        backL.setPower(gamepad1.left_stick_y);
        if (gamepad2.right_trigger >= .2) {
            flip.setPower(gamepad2.right_trigger);
        } else {
            flip.setPower(-gamepad2.left_trigger);
        }
        if (gamepad2.left_stick_y > 0) {
            intakeL.setPower(gamepad2.left_stick_y);
        } else {
            intakeL.setPower(.65 * gamepad2.left_stick_y);
        }
        if (gamepad2.right_stick_y > 0) {
            intakeR.setPower(gamepad2.right_stick_y);
        } else {
            intakeR.setPower(.65 * gamepad2.right_stick_y);
        }

        intakeR.setPower(.5*-gamepad2.left_stick_y);
        intakeL.setPower(.5*gamepad2.right_stick_y);
    }
    


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
    }

}