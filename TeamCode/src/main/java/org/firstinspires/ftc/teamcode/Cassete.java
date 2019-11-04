/*
Cassette, in this case, refers to the motor-driven portion of the robot, which was named such due to its resemblance of a cassette tape.
The "cassette" is a differential drive using two wheels. There are two motors, meaning that the two wheels must have the same angle and power at any point.
 */
package org.firstinspires.ftc.teamcode;
import android.os.SystemClock;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Cassete {

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
    private double currentTurnVelocity = 0; //current rate at which the module is turning

    private long currentTimeNanos = 0; //current time on the clock
    private long lastTimeNanos = 0; //previous update's clock time
    private double elapsedTimeThisUpdate = 0; //time of the update

    private double motor1Power = 0;
    private double motor2Power = 0;
    private String moduleName="nil";

    private ElapsedTime timeSinceStart = new ElapsedTime();
    private FtcDashboard dashboard;

    public Cassete(DcMotor motor1, DcMotor motor2, double angletoTurnAt,String cassetename) {
        this.topmotor = motor1;
        this.bottommotor = motor2;
        this.angleToTurnAt = angletoTurnAt;

        topmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // FLOAT means motor doesn't move or resist movement from outside forces
        bottommotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        topmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bottommotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    String basicTelemetry(){
        return (""+timeSinceStart.seconds() + "seconds in" + "Cassete" + " (" + moduleName + "): " + " Current variables angleError: " + angleError + " currentTargetAngle: " + currentTargetAngle + " currentTurnVelocity" + currentTurnVelocity);
    }





// F = ma
    void resetEncoders(){
        topmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottommotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottommotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
       private void applyPowers(){
        topmotor.setPower(motor1Power);
        bottommotor.setPower(motor2Power);
    }


    String getLogString(){
        return "top power: "+(Math.round(motor1Power*100.0))/100+", top encoder: "+topmotor.getCurrentPosition()+"\nbottom power: "+(Math.round(motor2Power*100.0))/100+"bottom encoder: "+bottommotor.getCurrentPosition();
    }
    void resetRuntime(){
        timeSinceStart.reset();
    }

    //the current error sum when turning toward the target
    private double turnErrorSum = 0;

    // PID LOOP!!!

    //Proportional, Integral, Differential

    private void calcPowers(double targetAnglerad,double wheelPower) {
        double Pgain=.05;
        double Igain=.075;
        double Dgain=.22;
        currentTimeNanos = SystemClock.elapsedRealtimeNanos();
        elapsedTimeThisUpdate = (currentTimeNanos - lastTimeNanos)/1e9;
        if(elapsedTimeThisUpdate < 0.003){
            return;//don't do anything if it is too fast
        }
        //remember the time to calculate delta the next update
        lastTimeNanos = currentTimeNanos;
        //if there has been an outrageously long amount of time, don't bother
        if(elapsedTimeThisUpdate > 1){
            return;
        }

        setHeading();
        angleError=subtractAngles(targetAnglerad,currentAngle_rad);
        //we should never turn more than 180 degrees, just reverse the direction
        while (Math.abs(angleError) > Math.toRadians(90)) {
            if(currentTargetAngle > currentAngle_rad){
                currentTargetAngle -= Math.toRadians(180);
            }else{
                currentTargetAngle += Math.toRadians(180);
            }
            wheelPower *= -1;
            angleError = subtractAngles(currentTargetAngle,currentAngle_rad);
        }
// TODO: UNDERSTAND EVERYTHING FROM HERE
        double angleErrorVelocity = angleError -
                ((getCurrentTurnVelocity() / Math.toRadians(300)) * Math.toRadians(30)
                        * Dgain);
        turnErrorSum += angleError * elapsedTimeThisUpdate;
        moduleRotationPower*= Range.clip(Math.abs(angleError)/Math.toRadians(2),0,1);


        moduleRotationPower = Range.clip((angleErrorVelocity / Math.toRadians(15)),-1,1)
                * Pgain;
        moduleRotationPower += turnErrorSum * Igain;

// TODO: TO HERE
        // if it has to rotate too much, you don't want the robot to run off.
        if(Math.abs(angleError)>20){
            wheelPower=0;
        }
        motor1Power = wheelPower*DiffCore.masterScale+moduleRotationPower*1.0;
        motor2Power = -(wheelPower*DiffCore.masterScale)+moduleRotationPower*1.0;
        //* 1.0 is to make sure it performs double multiplication.
        maximumPowerScale();
    }
    private void maximumPowerScale() {
        double scaleAmount = 1;
        if(motor1Power > 1 && motor1Power > motor2Power){
            scaleAmount = 1/motor1Power;
        }
        if(motor2Power > 1 && motor2Power > motor1Power){
            scaleAmount = 1/motor2Power;
        }
        motor1Power *= scaleAmount;
        motor2Power *= scaleAmount;
    }

    /**
     *  changed currentAngle_rad to the heading in radians from 0 to 2 PI
     */
    private void setHeading() {
        double encoderAvg = topmotor.getCurrentPosition() + bottommotor.getCurrentPosition() / 2;
        double reciprocal = 1 / HEADCONSTANT;
        double degreeHeading = ((reciprocal * (encoderAvg % HEADCONSTANT)) * 360);
        currentAngle_rad = Math.toRadians(degreeHeading);
    }
//expanded to make more clear, (1/constant)*(A+B) reduced

    /**
     * @param ang angle to begin with
     * @param subAng angle to subtract from ang
     * @return the difference in a positive number from 0 to 2 PI
     */
    private double subtractAngles(double ang,double subAng){
        double angle=ang-subAng;

        if(angle>(2*Math.PI)){
            return angle % (2*Math.PI);
        }
        return Math.abs(angle);
    }
    private double previousMeasureVelocityAngle = 0;
    //last time we updated the measure velocity
    private long lastMeasureVelocityTime = 0;
    private void calculateCurrentModuleRotationVelocity() {
        long currTime = SystemClock.uptimeMillis();
        if (currTime - lastMeasureVelocityTime > 40) {
            //measure the current turning speed of the module
            currentTurnVelocity = subtractAngles(currentAngle_rad, previousMeasureVelocityAngle) / ((currTime - lastMeasureVelocityTime) / 1000.0);

            previousMeasureVelocityAngle = currentAngle_rad;
            lastMeasureVelocityTime = currTime;
        }
    }
    private double getCurrentTurnVelocity() {
        return currentTurnVelocity;
    }
    void update(double targangle,double forwardpow){
        calculateCurrentModuleRotationVelocity();
        calcPowers(targangle,forwardpow);
        applyPowers();
    }

}
