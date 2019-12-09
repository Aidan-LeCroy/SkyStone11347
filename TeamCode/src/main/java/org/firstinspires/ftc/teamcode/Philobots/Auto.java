package org.firstinspires.ftc.teamcode.Philobots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DiffCore;
import org.firstinspires.ftc.teamcode.Tracking;

@Autonomous(name="WA Autonomous 1.0", group="Diffy")
@Disabled
public class Auto extends OpMode {

    Robot robot;
    Tracking track;
    @Override
    public void init() {
        track=new Tracking(hardwareMap,telemetry);
        track.initializeCamera();

    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        super.stop();
    }
}