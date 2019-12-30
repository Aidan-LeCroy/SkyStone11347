package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DiffCore;
import org.firstinspires.ftc.teamcode.Math.Vector;
import static org.firstinspires.ftc.teamcode.DiffConstants.MAX_SPEED;

@Autonomous(name="GK Auto", group="GK")
@Disabled
public class DiffAuto extends DiffCore {

    public void DiffAuto() {

    }
/*
    public void driveDistance(Vector dirmag, double speed, LinearOpMode linearOpMode) {
        double power = speed / MAX_SPEED;
        double moveTime = dirmag.getMagnitude() / speed;

        leftDrive.update(0, power);
        rightDrive.update(0, -power);
    }
*/
/*
    public void rotateAngle(double angle, double linearSpeed, LinearOpMode linearOpMode, boolean angleIsRadian) {
        angle *= (angleIsRadian) ? 1 : (Math.PI / 180);
        double power = linearSpeed / MAX_SPEED;
        double moveTime = angle /
    }
*/
}