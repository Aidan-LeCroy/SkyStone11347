package org.firstinspires.ftc.teamcode.TestDebugOpModes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@SuppressWarnings("ALL")
@TeleOp(name="getRPM",group="TestOpModes")
public class GetRPM extends OpMode {
    //find RPM, rotate each cassete
    private DcMotor motor1,motor2,motor3,motor4;
    private ElapsedTime time=new ElapsedTime();
    private long n=1;
    private long n2=1;
    double finT = 1;
    double num = 0;
    private Telemetry.Line line1,line2,line3,line4;
    public void init(){
        motor1=hardwareMap.dcMotor.get("topL");
        motor2=hardwareMap.dcMotor.get("bottomL");
        motor3=hardwareMap.dcMotor.get("topR");
        motor4=hardwareMap.dcMotor.get("bottomR");
        time.reset();
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);

//        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    public void loop(){
        setMotors(.75*gamepad1.left_stick_y);
        telemetry.addData("topL" + ": ", Double.toString(getTPS1(motor1)));
        telemetry.addData("bottomL" + ": ", Double.toString(getTPS1(motor2)));
        telemetry.addData("topR" + ": ", Double.toString(getTPS1(motor3)));
        telemetry.addData("bottomR" + ": ", Double.toString(getTPS1(motor4)));
    }


    private void getTPS(DcMotor motor, String wah){
        long currenttime=Math.round(time.seconds());
        if(currenttime>n2){
            telemetry.addLine("AverageTPS motor "+wah+": "+motor.getCurrentPosition()/n2);
            n2=currenttime;
        }
    }

    double getTPS1(DcMotor motor){
        if (finT - getRuntime() >= 1) {
            num = motor.getCurrentPosition()/(finT - getRuntime());
            finT = getRuntime();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return num;
    }

    private void setMotors(double power){
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
    }
}
