package org.firstinspires.ftc.teamcode.CircuitRunners;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems.Intake;
import org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems.LiftSystem;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import static java.lang.Math.abs;

public class Robot {
    DriveController driveController;
    private BNO055IMU imu;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    private OpMode opMode;
    private int liftLevel=1;
    private DcMotor leftIntake,rightIntake;
    private DcMotor leftLift,rightLift;
    private static final double ENCODER_TICKS_PER_LEVEL=200.0;
    private Servo leftFB,rightFB,grab;

    public LiftSystem liftSystem;
    public Intake intake;

    public boolean isAuto;

    public Gamepad gamepad1;
    public Gamepad gamepad2;

    public BulkDataManager bulkDataManager;
    ExpansionHubMotor topLeft,bottomLeft,topRight,bottomRight;

    public Robot (OpMode opMode, boolean isAuto, boolean headless) {

        this.isAuto = isAuto;
        this.hardwareMap = opMode.hardwareMap;
        initBulkData();
        initMechanisms();
        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;

        driveController = new DriveController(this,headless);
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
    }

    public void initBulkData() {
        bulkDataManager = new BulkDataManager(
                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 5"),
                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 7")
        );

    }

    public void initIMU () {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }
    //Left lift reversed, encoder ticks going negative.
    public void manLift(double power){
        leftLift.setPower(-power);
        rightLift.setPower(power);
    }
    private void initMechanisms(){

        leftIntake=hardwareMap.dcMotor.get("leftIntake");
        rightIntake=hardwareMap.dcMotor.get("rightIntake");
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFB=hardwareMap.servo.get("leftFB");
        rightFB=hardwareMap.servo.get("rightFB");
        rightFB.setDirection(Servo.Direction.REVERSE);
        rightLift=hardwareMap.dcMotor.get("rightLift");
        leftLift=hardwareMap.dcMotor.get("leftLift");
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grab=hardwareMap.servo.get("grab");
        grab.setDirection(Servo.Direction.REVERSE);
        liftSystem = new LiftSystem(this);
        intake = new Intake(this);

    }

    public void init4b(){
        leftFB.setPosition(0.03);
        rightFB.setPosition(0.03);
    }

    public Angle getRobotHeading () {
        //heading is of NEG_180_TO_180_HEADING type by default (no need for conversion)
        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        telemetry.addData("Robot Heading", heading);
        return new Angle(heading, Angle.AngleType.NEG_180_TO_180_HEADING);
    }

    public void wait (int millis, LinearOpMode linearOpMode) {
        long startTime = System.currentTimeMillis();
        while (millis > System.currentTimeMillis() - startTime && linearOpMode.opModeIsActive()) {}
    }

    public ExpansionHubMotor findMotor(String id){
        return hardwareMap.get(ExpansionHubMotor.class, id);
    }

    public AnalogInput findAnalog(String id){
        return hardwareMap.get(AnalogInput.class, id);
    }

    public Servo findServo(String id){
        return hardwareMap.get(Servo.class, id);
    }

    public void intake(double power){
        leftIntake.setPower(-power);
        rightIntake.setPower(power);
    }
    public void intakeOn() {
        this.intake(1);
    }
    public void intakeOff(){
        this.intake(0);
    }
    public void setGrabPos(double pos){
        grab.setPosition(pos);
    }
//    public void dropFoundation(){
//        leftSF.setPosition(1);
//        rightSF.setPosition(1);
//    }
//    public void liftFoundation(){
//        leftSF.setPosition(0);
//        rightSF.setPosition(0);
//
//    }

    public void set4BPos(double pos){
        leftFB.setPosition(pos);
        rightFB.setPosition(pos);
    }


}
