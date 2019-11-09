/*
Cassette, in this case, refers to the motor-driven portion of the robot, which was named such due to its resemblance of a cassette tape.
The "cassette" is a differential drive using two wheels. There are two motors, meaning that the two wheels must have the same angle and power at any point.
 */
package org.firstinspires.ftc.teamcode;
import android.os.SystemClock;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Locale;
class Cassette {

    private double currentTargetAngle = 0;
    private double angleToTurnAt;


    private double wheelPower;
    private double moduleRotationPower;

    private DcMotor topmotor;
    private DcMotor bottommotor;
    private double currentAngle_rad = 0; //real angle in radians
    private double angleError = 0;
    private double currentTurnVelocity = 0; //current rate at which the module is turning

    private long currentTimeNanos = 0; //current time on the clock
    private long lastTimeNanos = 0; //previous update's clock time

    private double motor1Power = 0;
    private double motor2Power = 0;
    private String moduleName;

    private ElapsedTime timeSinceStart = new ElapsedTime();
    Cassette(DcMotor motor1, DcMotor motor2, double angletoTurnAt, String cassettename) {
        this.topmotor = motor1;
        this.bottommotor = motor2;
        this.angleToTurnAt = angletoTurnAt;
        moduleName=cassettename;
        topmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // FLOAT means motor doesn't move or resist movement from outside forces
        bottommotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        topmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bottommotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    // %2.2f means 2 spaces to the accuracy of 2 decimal places. Numbers are right justified.
    String basicTelemetry() {
        return String.format(Locale.ENGLISH,"%d seconds in Cassette (%s) | Current Vars | " +
                "AngleError: %2.2f TargetAngle:%2.2f TurnVelocity:%2.2f Angle:%2.2f ",(int)timeSinceStart.seconds(),moduleName,
                Math.toDegrees(angleError),Math.toDegrees(currentTargetAngle),Math.toDegrees(currentTurnVelocity), Math.toDegrees(currentAngle_rad))
                +String.format(Locale.ENGLISH,"ModuleRotationPower %1.2f WheelPower %1.2f",moduleRotationPower,wheelPower);
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
        return String.format(Locale.ENGLISH,"top power: %2.2f bottom power: %2.2f \ntop encoder: %d bottom encoder: %d",motor1Power,motor2Power,topmotor.getCurrentPosition(),bottommotor.getCurrentPosition());
    }
    void resetRuntime(){
        timeSinceStart.reset();
    }

    //the current error sum when turning toward the target
    private double turnErrorSum = 0;

    //Proportional, Integral, Differential

    private void    calcPowers(double targetAngle_rad,double wheelPower1) {
        wheelPower=wheelPower1;
        currentTargetAngle=targetAngle_rad;
        //PID coefficients
        double Pgain=.05;
        double Igain=0.0;//.075
        double Dgain=0.0;//.22
        currentTimeNanos = SystemClock.elapsedRealtimeNanos();
        double elapsedTimeThisUpdate = (currentTimeNanos - lastTimeNanos)/1e9;
        if(elapsedTimeThisUpdate < 0.003){
            return;
        }
        //remember time
        lastTimeNanos = currentTimeNanos;
        //if time elapsed is greater than 1 second, just stop.
        if(elapsedTimeThisUpdate > 1){
            return;
        }
        setHeading();
        angleError=subtractAngles(targetAngle_rad,currentAngle_rad);
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
        //derivative
        turnErrorSum += angleError * elapsedTimeThisUpdate;
       moduleRotationPower*= Range.clip(Math.abs(angleError)/Math.toRadians(2),0,1);

//        proportional
        moduleRotationPower = Range.clip((angleErrorVelocity / Math.toRadians(15)),-1,1)
                * Pgain;
//        Integral
        moduleRotationPower += turnErrorSum * Igain;

// TODO: TO HERE
        // if it has to rotate too much, you don't want the robot to run off.
        if(Math.toDegrees(Math.abs(angleError))>20){
            wheelPower=0;
        }
        wheelPower*=DiffCore.masterScale;
        motor1Power = wheelPower+moduleRotationPower*1.0;
        motor2Power = -wheelPower+moduleRotationPower*1.0;
        //*1.0 is to make sure it performs double multiplication.
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
    private void setHeading(){
        double encoderAvg = topmotor.getCurrentPosition() + bottommotor.getCurrentPosition() / 2.0;
        double reciprocal = 1 / DiffConstants.HEADCONSTANT;
        double degreeHeading = ((reciprocal * (encoderAvg % DiffConstants.HEADCONSTANT)) * 360);
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

        if(angle>(2.0*Math.PI)){
            return angle % (2.0*Math.PI);
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
