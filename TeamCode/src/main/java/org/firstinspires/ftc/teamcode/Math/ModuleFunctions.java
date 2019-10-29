package org.firstinspires.ftc.teamcode.Math;

public class ModuleFunctions {

    public static final double ENCODERTICKPERREV=305;

    public static double calculateAngle(double topPos,double botPos){
        double degAngle = (((((topPos+botPos)/2)%ENCODERTICKPERREV)/ENCODERTICKPERREV)*360);
        return Math.toRadians(degAngle);
    }
    // TODO: Figure out if this method is correct
    public static double subtractAngles(double angle1,double angle2){
        return Math.abs(angle2-angle1);
    }
}
