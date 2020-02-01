package org.firstinspires.ftc.teamcode.CircuitRunners;

import org.firstinspires.ftc.robotcore.external.Func;

public  class RobotUtil {

    public static double scaleVal (double input, double minInputVal,
                                   double maxInputVal, double minOutputVal, double maxOutputVal) {
        if (input > maxInputVal) input = maxInputVal;
        double inputRange = Math.abs(maxInputVal - minInputVal);
        double outputRange = Math.abs(maxOutputVal - minOutputVal);
        double scaleFactor = input/inputRange;
        return outputRange * scaleFactor + minOutputVal;
    }
    public static double toCm(double inches) { return inches * 2.54; }
    public static double toIn(double centimetres) { return centimetres / 2.54; }
}
