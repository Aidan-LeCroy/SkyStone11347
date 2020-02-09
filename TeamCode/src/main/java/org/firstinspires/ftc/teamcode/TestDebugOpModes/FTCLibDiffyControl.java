package org.firstinspires.ftc.teamcode.TestDebugOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveModuleEx;
import com.arcrobotics.ftclib.hardware.motors.MotorImplEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import  com.arcrobotics.ftclib.drivebase.swerve.DiffySwerveDrive;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.CircuitRunners.DriveModule;

public class FTCLibDiffyControl extends LinearOpMode {


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

    private static final String SLOW_MODE_REDUCTION_NAME = "Driver Slow-Mode Multiplier";

    private double kWheelLeft = DriveModule.CM_PER_TICK;
    private double kWheelRight = DriveModule.CM_PER_TICK;

    private double kAngleLeft = 0.8;
    private double kAngleRight = 0.8;

    private double slowModeConst = 0.75;







    //HARDWARE

    private DiffySwerveDrive diffySwerveDrive; //Drive base object

    private MotorImplEx topLeft, bottomLeft, topRight, bottomRight;

    private DiffySwerveModuleEx moduleLeft, moduleRight;



    private final double driveCPR = 28 * 4 * 15.9;


    @Override
    public void runOpMode(){

        topLeft = new MotorImplEx(hardwareMap, "topL", driveCPR);
        bottomLeft = new MotorImplEx(hardwareMap, "bottomL", driveCPR);
        topRight = new MotorImplEx(hardwareMap, "topR", driveCPR);
        bottomRight = new MotorImplEx(hardwareMap, "bottomR", driveCPR);


        moduleLeft = new DiffySwerveModuleEx(topLeft,bottomLeft, kAngleLeft, kWheelLeft);
        moduleRight = new DiffySwerveModuleEx(topRight,bottomRight, kAngleRight, kWheelRight);

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
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.right_stick_y
                );
            } else {
                diffySwerveDrive.drive(
                        gamepad1.left_stick_x * slowModeConst,
                        -gamepad1.left_stick_y * slowModeConst,
                        gamepad1.right_stick_x * slowModeConst,
                        gamepad1.right_stick_y * slowModeConst
                );
            }
            packet.put("Left Module Vector X", gamepad1.left_stick_x);
            packet.put("Left Module Vector Y", -gamepad1.left_stick_y);
            packet.put("Right Module Vector X", gamepad1.right_stick_x);
            packet.put("Right Module Vector Y", gamepad1.right_stick_y);
            packet.put("Slow-Mode Active", gamepad1.right_bumper);
            packet.put("Loop Time (ms)", benchmark.milliseconds());
            benchmark.reset();

            dashboard.sendTelemetryPacket(packet);


        }
        diffySwerveDrive.stopMotor();
        diffySwerveDrive.disable();






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
        classVar.putVariable(MODULE_K_WHEEL_NAME, kWheels);
        classVar.putVariable(MODULE_K_ANGLE_NAME, kAngles);
        classVar.putVariable(SLOW_MODE_REDUCTION_NAME, slowModeMult);
        dashboard.updateConfig();
    }
}
