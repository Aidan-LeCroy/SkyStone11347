/*
Cassette, in this case, refers to the motor-driven portion of the robot, which was named such due to its resemblance of a cassette tape.
The "cassette" is a differential drive using two wheels. There are two motors, meaning that the two wheels must have the same angle and power at any point.
 */
package org.firstinspires.ftc.teamcode;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

public class Cassete {

    private double topEncoder;
    private double bottomEncoder;
    private double currentTargetAngle = 0;

    private double angleToTurnAt = 0;

    private static final double HEADCONSTANT = (41888.0/192.0);

    private double wheelPower;
    private double moduleRotationPower;

    private DcMotor topmotor;
    private DcMotor bottommotor;


    private double currentAngle_rad = 0; //real angle in radians
    private double previousAngle_rad = 0; // angle from previous update, used for velocity

    private double angleError = 0;
    private double turnPower = 0; //not user input, calculated based on error
    private double currentTurnVelocity = 0; //current rate at which the module is turning

    private long currentTimeNanos = 0; //current time on the clock
    private long lastTimeNanos = 0; //previous update's clock time
    private double elapsedTimeThisUpdate = 0; //time of the update

    private double motor1Power = 0;
    private double motor2Power = 0;

    private double turnErrorSum = 0;
    private String moduleName;

    boolean shouldILog = true;
    double currenttime = 0;
    private ElapsedTime timeSinceStart = new ElapsedTime();

    public Cassete(DcMotor motor1, DcMotor motor2, double angletoTurnAt,String cassetename) {
        this.topmotor = motor1;
        this.bottommotor = motor2;
        this.angleToTurnAt = angletoTurnAt;

        topmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // FLOAT means motor doesn't move or resist movement from outside forces
        bottommotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        topmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bottommotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }




    public void setSpeedMagnitude(double mag){
        wheelPower = mag;
    }
    public void resetEncoders(){
        topmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottommotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
       private void applyPowers(){
        topmotor.setPower(motor1Power);
        bottommotor.setPower(motor2Power);
    }
    void robotLog(){
        if((timeSinceStart.milliseconds()-50)>currenttime){
        return;
        }
        RobotLog.d("Cassete", timeSinceStart.seconds() + "seconds in" + "Cassete" + " (" + moduleName + "): " + " Current variables angletoturnat: " + angleToTurnAt + " currentTargetAngle: " + currentTargetAngle + " currentTurnVelocity" + currentTurnVelocity);
        currenttime = timeSinceStart.milliseconds();
    }
    String getLogString(){
        return "top power: "+motor1Power+", top encoder: "+topmotor.getCurrentPosition()+"\nbottom power: "+motor2Power+"bottom encoder: "+bottommotor.getCurrentPosition();
    }
    public void resetRuntime(){
        timeSinceStart.reset();
    }
    public void setTargetHeading(double heading){
        currentTargetAngle = heading;
    }
    public void moduleRotationPID(){
    }
    public void updatePower() {
        motor1Power = Range.clip(moduleRotationPower + wheelPower, -1, 1);
        motor2Power = Range.clip(moduleRotationPower - wheelPower, -1, 1);
    }
    public double getHeading(){
        double encoderAvg = topmotor.getCurrentPosition()+bottommotor.getCurrentPosition()/2;
        double reciprocal = 1/HEADCONSTANT;
        double degreeHeading = ((reciprocal*(encoderAvg%HEADCONSTANT))*360);
        return Math.toRadians(degreeHeading);
    }
    //expanded to make more clear, (1/constant)*(A+B) reduced
}

