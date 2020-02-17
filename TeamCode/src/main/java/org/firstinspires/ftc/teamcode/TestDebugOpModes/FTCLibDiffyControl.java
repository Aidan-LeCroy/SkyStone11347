package org.firstinspires.ftc.teamcode.TestDebugOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveModuleEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorImplEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import  com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.CircuitRunners.DriveModule;
import org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems.AS5600;

import java.util.ArrayList;
import java.util.Locale;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems.Intake;
import org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems.LiftSystem;

@Config
@TeleOp(group = "FTCLib TeleOP")
public class FTCLibDiffyControl extends LinearOpMode {

    //Dashboard IP: 192.168.49.1:8080/dash

    //Dashboard things
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    //The class (The .java file we are in) name and variable that all of this shows up under
    private String className;
    private CustomVariable classVar;

    private static final String MODULE_K_WHEEL_NAME = "Diffy Module kWheel";

    private static final String LEFT_MODULE_K_WHEEL_NAME = "Left Module kWheel";
    private static final String RIGHT_MODULE_K_WHEEL_NAME = "Right Module kWheel";

    private static final String MODULE_K_ANGLE_NAME = "Diffy Module kAngle";

    private static final String LEFT_MODULE_K_ANGLE_NAME = "Left Module kAngle";
    private static final String RIGHT_MODULE_K_ANGLE_NAME = "Right Module kAngle";
    
    private static final String MODULE_ROTATION_P_NAME = "Diffy Module Rotational P";
    
    private static final String LEFT_MODULE_ROTATION_P_NAME = "Left Module Rotational P";
    private static final String RIGHT_MODULE_ROTATION_P_NAME = "Right Module Rotational P";

    private static final String SLOW_MODE_REDUCTION_NAME = "Driver Slow-Mode Multiplier";

    private static final String[] driveMotorIds = {"topL", "bottomL", "topR", "bottomR"};

    private double kWheelLeft = DriveModule.CM_PER_TICK;
    private double kWheelRight = DriveModule.CM_PER_TICK;

    private double kAngleLeft = 0.8;
    private double kAngleRight = 0.8;
    
    private double rotationPLeft = 0.61923568104;
    private double rotationPRight = 0.61923568104;

    private double slowModeConst = 0.75;





    PIDFController defaultRevMotorVelo = new PIDFController(new double[] {1.17, 0.117, 0, 11.7});
    PController defaultRevMotorPos = new PController(5);


    //Controls
    private boolean grabbing=false;
    private boolean bounce1 = false;
    private boolean bounce2 = false;
    private boolean bounce3= false;
    private boolean fbDrop=false;


    //HARDWARE

    private Servo leftFB,rightFB,grab;

    private DiffySwerveDrive diffySwerveDrive; //Drive base object

    private MotorImplEx topLeft, bottomLeft, topRight, bottomRight;

    private AS5600 Lmagnet;
    private AS5600 Rmagnet;

    private DiffySwerveModuleEx moduleLeft, moduleRight;

    private static final double L_MAGNET_OFFSET = 0;
    private static final double R_MAGNET_OFFSET = 0;

    private DoubleSupplier headingSensorL = () -> {
        double raw = (Lmagnet.getHeading() + L_MAGNET_OFFSET);
        if(raw < 0) raw += 360;
        if(raw > 360) raw -= 360;
        return AngleUnit.DEGREES.normalize(raw);
    };

    private DoubleSupplier headingSensorR = () -> {
        double raw = (Rmagnet.getHeading() + R_MAGNET_OFFSET);
        if(raw < 0) raw += 360;
        if(raw > 360) raw -= 360;
        return AngleUnit.DEGREES.normalize(raw);
    };



    private static final double driveCPR = 28 * 4 * 15.9;

    private ArrayList<MotorImplEx> driveMotors = new ArrayList<>();

    //Testing for mechanisms
    private Intake intake = new Intake(this);
    private LiftSystem lift = new LiftSystem(this);


    @Override
    public void runOpMode(){
        Lmagnet = new AS5600(hardwareMap.analogInput.get("Lmagnet"));
        Rmagnet = new AS5600(hardwareMap.analogInput.get("Rmagnet"));



        topLeft = new MotorImplEx(hardwareMap, driveMotorIds[0], driveCPR, defaultRevMotorVelo, defaultRevMotorPos);
        bottomLeft = new MotorImplEx(hardwareMap, driveMotorIds[1], driveCPR, defaultRevMotorVelo, defaultRevMotorPos);
        topRight = new MotorImplEx(hardwareMap, driveMotorIds[2], driveCPR, defaultRevMotorVelo, defaultRevMotorPos);
        bottomRight = new MotorImplEx(hardwareMap, driveMotorIds[3], driveCPR, defaultRevMotorVelo, defaultRevMotorPos);

        leftFB=hardwareMap.servo.get("leftFB");
        rightFB=hardwareMap.servo.get("rightFB");
        rightFB.setDirection(Servo.Direction.REVERSE);

        grab=hardwareMap.servo.get("grab");
        grab.setDirection(Servo.Direction.REVERSE);

        driveMotors.add(topLeft);
        driveMotors.add(bottomLeft);
        driveMotors.add(topRight);
        driveMotors.add(bottomRight);

        for(MotorImplEx motor : driveMotors){
            motor.setMode(MotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(MotorEx.RunMode.RUN_USING_ENCODER);
        }


        moduleLeft = new DiffySwerveModuleEx(topLeft,bottomLeft, kAngleLeft, kWheelLeft, rotationPLeft);
        moduleRight = new DiffySwerveModuleEx(topRight,bottomRight, kAngleRight, kWheelRight, rotationPRight);

        moduleLeft.setHeadingInterpol(headingSensorL);
        moduleRight.setHeadingInterpol(headingSensorR);

        diffySwerveDrive = new DiffySwerveDrive(moduleLeft, moduleRight);

        diffySwerveDrive.setRightSideInverted(true);
        diffySwerveDrive.stopMotor();

        constructVariables();

        ElapsedTime benchmark = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        waitForStart();

        benchmark.reset();
        while (opModeIsActive()){



            TelemetryPacket packet = new TelemetryPacket();

            if(!gamepad1.right_bumper) {
                diffySwerveDrive.drive(
                        -gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        -gamepad1.right_stick_x,
                        gamepad1.right_stick_y
                );
            } else {
                diffySwerveDrive.drive(
                        -gamepad1.left_stick_x * slowModeConst,
                        -gamepad1.left_stick_y * slowModeConst,
                        -gamepad1.right_stick_x * slowModeConst,
                        gamepad1.right_stick_y * slowModeConst
                );
            }


            intake.update();
            lift.update();


            packet.put("Left Module Vector X", gamepad1.left_stick_x);
            packet.put("Left Module Vector Y", -gamepad1.left_stick_y);
            packet.put("Right Module Vector X", gamepad1.right_stick_x);
            packet.put("Right Module Vector Y", gamepad1.right_stick_y);
            packet.put("Slow-Mode Active", gamepad1.right_bumper);
            packet.put("Loop Time", String.format(Locale.US ,"%.2f ms", benchmark.milliseconds()));
            benchmark.reset();

            dashboard.sendTelemetryPacket(packet);

        }
        diffySwerveDrive.stopMotor();
        diffySwerveDrive.disable();

    }

    //Just so we don't take up space
    private void moveThings(){
        if(gamepad2.a&&!bounce3){
            flip4B();
            bounce3=true;
        }
        else if(!gamepad2.a){
            bounce3=false;
        }

        if(gamepad2.left_bumper&&!bounce1){
            grabToggle();
            bounce1=true;
        }
        else if(!gamepad2.left_bumper){
            bounce1=false;
        }

        if(gamepad2.y){
            leftFB.setPosition(.95);
            leftFB.setPosition(.95);
        }
    }

    private void set4BPos(double pos){
        leftFB.setPosition(pos);
        rightFB.setPosition(pos);
    }

    private void flip4B(){
        if(!fbDrop){
            set4BPos(.03);
            fbDrop=true;
        }
        else if(fbDrop){
            set4BPos(.85);
            fbDrop=false;
        }
    }
    private void grabToggle(){
        if(!grabbing){
            grab.setPosition(0.5);
            grabbing=true;
        }
        else if(grabbing){
            grab.setPosition(0.6);
            grabbing=false;
        }
    }

    private void constructVariables(){
        className = getClass().getSimpleName();
        classVar = (CustomVariable) dashboard.getConfigRoot().getVariable(className);
        if (classVar == null) {
            // this should never happen...
            classVar = new CustomVariable();
            dashboard.getConfigRoot().putVariable(className, classVar);

            RobotLog.w("Unable to find top-level category %s", className);
        }

        CustomVariable kWheels = new CustomVariable();
        CustomVariable kAngles = new CustomVariable();
        CustomVariable rotationalP = new CustomVariable();
        CustomVariable slowModeMult = new CustomVariable();

        kWheels.putVariable(LEFT_MODULE_K_WHEEL_NAME, new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return kWheelLeft;
            }

            @Override
            public void set(Double value) {
                kWheelLeft = value;
                moduleLeft.kWheelConstant = value;
            }
        }));
        kWheels.putVariable(RIGHT_MODULE_K_WHEEL_NAME, new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return kWheelRight;
            }

            @Override
            public void set(Double value){
                kWheelRight = value;
                moduleRight.kWheelConstant = value;
            }
        }));
        kAngles.putVariable(LEFT_MODULE_K_ANGLE_NAME, new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get(){
                return kAngleLeft;
            }

            @Override
            public void set(Double value){
                kAngleLeft = value;
                moduleLeft.kRevConstant = value;
            }
        }));
        kAngles.putVariable(RIGHT_MODULE_K_ANGLE_NAME, new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return kAngleRight;
            }

            @Override
            public void set(Double value) {
                kAngleRight = value;
                moduleRight.kRevConstant = value;

            }
        }));
        slowModeMult.putVariable(SLOW_MODE_REDUCTION_NAME, new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return slowModeConst;
            }

            @Override
            public void set(Double value) {
                slowModeConst = value;
            }
        }));
        rotationalP.putVariable(LEFT_MODULE_ROTATION_P_NAME, new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return rotationPLeft;
            }

            @Override
            public void set(Double value) { //Unfortunately we have to do this
                rotationPLeft = value;
                moduleLeft = new DiffySwerveModuleEx(topLeft,bottomLeft, kAngleLeft, kWheelLeft, rotationPLeft); // I hate it too...
                
            }
        }));
        rotationalP.putVariable(RIGHT_MODULE_ROTATION_P_NAME, new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return rotationPRight;
            }

            @Override
            public void set(Double value) { //Unfortunately we have to do this
                rotationPRight = value;
                moduleLeft = new DiffySwerveModuleEx(topLeft,bottomLeft, kAngleLeft, kWheelLeft, rotationPRight); // I hate it too...
                
            }
        }));
        classVar.putVariable(MODULE_K_WHEEL_NAME, kWheels);
        classVar.putVariable(MODULE_K_ANGLE_NAME, kAngles);
        classVar.putVariable(MODULE_ROTATION_P_NAME, rotationalP);
        classVar.putVariable(SLOW_MODE_REDUCTION_NAME, slowModeMult);
        dashboard.updateConfig();
    }
}
