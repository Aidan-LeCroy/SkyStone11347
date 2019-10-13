package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

public class Cassete {
    private double wheelAngle;
    private double topEncoder;
    private double bottomEncoder;
    private double currentTargetAngle = 0;
    private double angleToTurnAt = 0;
    private double currentForwardsPower = 0;
    private DcMotor topmotor;
    private DcMotor bottommotor;

    public Cassete(DcMotor motor1, DcMotor motor2) {
        this.topmotor = motor1;
        this.bottommotor = motor2;
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public double getWheelAngle() {
        return wheelAngle;
    }

    public double getDeltaEncoder(DcMotor top, DcMotor bottom) {
        return ((top.getCurrentPosition() + bottom.getCurrentPosition()) / 2.0);
    }

    public void setDriveTrainDirection(double amountForwards, double amountSideWays,
                                       double amountTurn) {

        double xComponent = amountForwards * 1 + amountSideWays * 0 +
                Math.cos(angleToTurnAt) * amountTurn;
        double yComponent = amountForwards * 0 + amountSideWays * 1 +
                Math.sin(angleToTurnAt) * amountTurn;


        currentForwardsPower = Math.hypot(xComponent, yComponent);
        if (Math.abs(currentForwardsPower) > 0.03) {
            currentTargetAngle = Math.atan2(yComponent, xComponent);
        }
    }

        public void resetEncoders(){
            topmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottommotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

