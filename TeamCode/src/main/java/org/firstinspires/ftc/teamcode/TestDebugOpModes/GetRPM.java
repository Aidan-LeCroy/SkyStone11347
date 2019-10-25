package org.firstinspires.ftc.teamcode.TestDebugOpModes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Cassete;
@TeleOp(name="getRPM",group="TestOpModes")
public class GetRPM extends OpMode {
    //find RPM, rotate each cassete
    private DcMotor motor1,motor2,motor3,motor4;
    private ElapsedTime time=new ElapsedTime();
    private long n=1;
    private long n2=1;
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
        setMotors(gamepad1.left_stick_y);
        getTPS(motor1,line1,"topL");
        getTPS(motor2,line2,"bottomL");
        getTPS(motor3,line3,"topR");
        getTPS(motor4,line4,"bottomR");
    }


    private void getTPS(DcMotor mot,Telemetry.Line line,String wah){
        long currenttime=Math.round(time.seconds());
        if(currenttime>n2){
            line=telemetry.addLine("AverageTPS motor "+wah+": "+mot.getCurrentPosition()/n2);
            n2=currenttime;
        }
    }
    private void setMotors(double power){
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
    }
}
