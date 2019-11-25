package org.firstinspires.ftc.teamcode.Math;

public class DiffPos {
    public DiffPos(double xPos, double yPos, double thetaPos, boolean isRadian) {
        if (!isRadian) {
            thetaPos = Math.toRadians(thetaPos);
        }
        Vector position = new Vector(xPos, yPos);
        Pose pose = new Pose(position, thetaPos);
    }
    public trackMovement(Pose selfpos) {
        double x = selfpos.getVec().getX();
        double y = selfpos.getVec().getY();
        double theta = selfpos.getTheta();
        wait(long 1);
        double newX = selfpos.getVec().getX();
        double newY = selfpos.getVec().getY();
        double newTheta = selfpos.getTheta();
        Vector velocity = new Vector(1000 * (newX - x), 1000 * (newY - y));
    }
}

