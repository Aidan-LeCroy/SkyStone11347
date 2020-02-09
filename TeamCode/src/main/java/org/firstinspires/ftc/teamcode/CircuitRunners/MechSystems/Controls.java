package org.firstinspires.ftc.teamcode.CircuitRunners.MechSystems;


import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.CircuitRunners.Robot;

/**
 * Class to hold the controls for mainly the manipulator.
 *
 * Probably a dumb idea, but it could hopefully cut down on all the mess
 * involved with reading gamepad inputs in other classes
 */
public class Controls {

    private Gamepad gamepad2;
    private Gamepad gamepad1;

    public Controls(Robot robot){
        this.gamepad1 = robot.gamepad1;
        this.gamepad2 = robot.gamepad2;
    }

    //Control for lift up
    public final Func<Boolean> liftUp = () -> gamepad2.dpad_up;

    //Control for lift down
    public final Func<Boolean> liftDown = () -> gamepad2.dpad_down;

    //Control for lift bottom extension
    public final Func<Boolean> liftBottom = () -> gamepad2.dpad_left || gamepad2.dpad_right;

    //Toggle for nub grabber
    public final Func<Boolean> grabberMove = () -> gamepad2.left_bumper;

    //Toggle for v4b
    public final Func<Boolean> v4barMove = () -> gamepad2.a;

    //Toggle for foundation grabber
    public final Func<Boolean> foundationMove = () -> gamepad2.x;

    //Control for intake in
    public final Func<Boolean> intakeIn = () -> gamepad2.left_trigger >= 0.05;

    //Control for intake out
    public final Func<Boolean> intakeOut = () -> gamepad2.right_trigger >= 0.05;

    //Control to move v4bar to .95 pos????
    public final Func<Boolean> v4barExternalMove = () -> gamepad2.y;

    //Control for driver slow-mode
    public final Func<Boolean> slowMode = () -> gamepad1.right_bumper;

    //X and Y for left module (differential drive) - FTCLib Vector2d
    public final Func<Vector2d> leftModuleVector = () -> new Vector2d(
            gamepad1.left_stick_x,
            -gamepad1.left_stick_y
    );

    //X and Y for right module (differential drive) - FTCLib Vector2d
    public final Func<Vector2d> rightModuleVector = () -> new Vector2d(
            gamepad1.right_stick_x,
            gamepad1.right_stick_y
    );

    //translational vector (arcade drive) - FTCLib Vector2d
    public final Func<Vector2d> translationalVector = () -> new Vector2d(
            gamepad1.left_stick_x,
            -gamepad1.left_stick_y
    );

    //The rotational vector (really just the right X and 0 for Y) (arcade drive) - FTCLib Vector2d
    public final Func<Vector2d> rotationalVector = () -> new Vector2d(
            gamepad1.right_stick_x,
            0 // nothing here
    );





}
