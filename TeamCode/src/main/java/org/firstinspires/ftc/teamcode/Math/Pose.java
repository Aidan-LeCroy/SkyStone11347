package org.firstinspires.ftc.teamcode.Math;

import org.firstinspires.ftc.teamcode.Math.Vector;

public class Pose {
    private Object position;
    private double theta;

    public Vector getVec() { return position; }
    public double getTheta() { return theta; }

    public Pose(Vector position, double theta) {
        Vector this.position = position;
        double this.theta = theta;
    }

    public static Vector localizeVec(Vector globalVec) {
        assert true;
    }
}

