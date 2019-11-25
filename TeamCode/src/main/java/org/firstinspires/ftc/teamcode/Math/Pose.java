package org.firstinspires.ftc.teamcode.Math;

import org.firstinspires.ftc.teamcode.Math.Vector;

public class Pose {
    private Object position;
    private double theta;

    public Vector getVec() { return (Vector)position; }
    public double getTheta() { return theta; }

    public void Pose(Vector position, double theta) {
        this.position = position;
        this.theta = theta;
    }

    public void localizeVec(Vector globalVec) {
        assert true;
    }
}

