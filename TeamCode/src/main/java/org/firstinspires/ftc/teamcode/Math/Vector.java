package org.firstinspires.ftc.teamcode.Math;

public class Vector {
    private final double x;
    private final double y;

    public Vector(double x, double y){ // initializes a vector with an x and y component
        this.x = x;
        this.y = y;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getMagnitude(){
        return Math.sqrt((x*x)+(y*y)); // returns the magnitude of the vector using the Pythagorean Theorem
    }
    public double getAngle() {
        return (180/Math.PI)*Math.atan2(y,x); // returns the angle of the vector on the standard unit circle in radians
    }
    public Vector add(Vector vec){
        return new Vector(x + vec.getX(),y + vec.getY()); // combines two vectors
    }
    public Vector scale(double scaleFactor){
        return new Vector(getX() * scaleFactor,getY() * scaleFactor); // scales the vector by a specified factor(aka multiplication)
    }


}
