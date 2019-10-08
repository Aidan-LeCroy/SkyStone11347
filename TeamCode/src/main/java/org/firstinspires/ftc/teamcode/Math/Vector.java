package org.firstinspires.ftc.teamcode.Math;

public class Vector {
    private final double x;
    private final double y;

    public Vector(double x, double y){
        this.x=x;
        this.y=y;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getMagnitude(){
        return Math.sqrt((x*x)+(y*y));
    }
    public double getAngle() {
        return (180/Math.PI)*Math.atan2(y,x);
    }
    public Vector add(Vector vec){
        return new Vector(x+vec.getX(),y+vec.getY());
    }
    public Vector scale(double scaleFactor){
        return new Vector(getX()*scaleFactor,getY()*scaleFactor);
    }


}
