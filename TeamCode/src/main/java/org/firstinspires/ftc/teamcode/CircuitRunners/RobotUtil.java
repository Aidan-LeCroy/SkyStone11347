package org.firstinspires.ftc.teamcode.CircuitRunners;

import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Paint;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;

public  class RobotUtil {

    private Resources res;
    private Bitmap robotPic;
    private android.graphics.Canvas canvas;

    private Paint paint = new Paint();

    private static final int robotPicId = R.drawable.shuckstack;

    private Vector2d robotCenter = new Vector2d(0, 0); //The basis of the drawing
    private Vector2d headingLine = new Vector2d(0, 100); //The line that signifies the front


    public static double scaleVal (double input, double minInputVal,
                                   double maxInputVal, double minOutputVal, double maxOutputVal) {
        if (input > maxInputVal) input = maxInputVal;
        double inputRange = Math.abs(maxInputVal - minInputVal);
        double outputRange = Math.abs(maxOutputVal - minOutputVal);
        double scaleFactor = input/inputRange;
        return outputRange * scaleFactor + minOutputVal;
    }
    public double normalizeTo360(double heading){
        while(!(heading > 360) && !(heading < 0)){
            if(heading > 360) heading -= 360;
            if(heading < 0) heading += 360;
        }
        return heading;
    }

    public static double toCm(double inches) { return inches * 2.54; }
    public static double toIn(double centimetres) { return centimetres / 2.54; }





    public void setRes(Resources res){
        this.res = res;
    }

    public void loadRes(){
        robotPic = BitmapFactory.decodeResource(res, robotPicId);
        canvas = new android.graphics.Canvas(robotPic);
    }

    public void updateHeading(double heading){
        double rotateCCW = 360 - (normalizeTo360(heading) - lastKnownHeading);
        headingLine.rotateBy(rotateCCW);
        canvas.drawLine(
                canvas.getWidth() / 2,
                canvas.getHeight() / 2,
                (float) headingLine.getX(),
                (float) headingLine.getY(),
                paint
        );
        lastKnownHeading = heading;
    }

    public Bitmap getRobotPic() {
        return robotPic;
    }

    //Things for drawing the robot and it's orientation on the FTC dashboard field overlay

    //This "SHOULD" work.... I spent a while on it because it was fun.
    //It will be a horrible image, mind. I'm not an artist.
    private Canvas imageOverlay;

    //The outline of the robot
    private List<Vector2d> robotOutline = constructRobotOutline();
    private List<Vector2d> robotRectFeature = constructRobotRectFeature();

    private double lastKnownHeading = 0;
    public void setImageOverlay(Canvas imageOverlay){
        this.imageOverlay = imageOverlay;
    }

    public Canvas getRobotImage(Canvas canvas, double heading){
        double rotateCCW = 360 - (normalizeTo360(heading) - lastKnownHeading);
        lastKnownHeading = heading; //Save the heading
        canvas.setStroke("#464646"); //Set to dark grey
        canvas.setStrokeWidth(1);
        double[] robotX = new double[robotOutline.size()];
        double[] robotY = new double[robotOutline.size()];
        double[] rectX = new double[robotRectFeature.size()];
        double[] rectY = new double[robotRectFeature.size()];
        robotOutline.forEach(
                (Vector2d point) -> {
                    point
                            .rotateBy(rotateCCW)
                            .div(2);
                }
        );
        robotRectFeature.forEach(
                (Vector2d point) -> {
                    point
                            .rotateBy(rotateCCW)
                            .div(2);
                }
        );
        Vector2d frontLine = new Vector2d(0, 10).rotateBy(rotateCCW);
        for(int i=0;i<robotOutline.size();i++){
            robotX[i] = robotOutline.get(i).getX();
            robotY[i] = robotOutline.get(i).getY();
        }
        for(int i=0;i<robotRectFeature.size();i++){
            rectX[i] = robotRectFeature.get(i).getX();
            rectY[i] = robotRectFeature.get(i).getY();
        }
        canvas.strokePolygon(robotX, robotY);
        canvas.setStroke("#3BC20E");
        canvas.setFill("#3BC20E");
        canvas.fillPolygon(rectX, rectY);
        canvas.setStroke("#FF1303");
        canvas.strokeLine(0, 0, frontLine.getX(), frontLine.getY());
        return canvas;
    }



    //Just to save space
    private List<Vector2d> constructRobotOutline(){
        List<Vector2d> outline = new ArrayList<>();
        outline.add(new Vector2d(
                robotCenter.getX() - 48,
                robotCenter.getY() - 36
        ));
        outline.add(new Vector2d(
                robotCenter.getX() - 48,
                robotCenter.getY() - 60
        ));
        outline.add(new Vector2d(
                robotCenter.getX() - 60,
                robotCenter.getY() - 60
        ));
        outline.add(new Vector2d(
                robotCenter.getX() - 60,
                robotCenter.getY() + 60
        ));
        outline.add(new Vector2d(
                robotCenter.getX() - 48,
                robotCenter.getY() + 60
        ));
        outline.add(new Vector2d(
                robotCenter.getX() - 48,
                robotCenter.getY() + 36
        ));
        outline.add(new Vector2d(
                robotCenter.getX() + 48,
                robotCenter.getY() + 36
        ));
        outline.add(new Vector2d(
                robotCenter.getX() + 48,
                robotCenter.getY() + 60
        ));
        outline.add(new Vector2d(
                robotCenter.getX() + 60,
                robotCenter.getY() + 60
        ));
        outline.add(new Vector2d(
                robotCenter.getX() + 60,
                robotCenter.getY() - 60
        ));
        outline.add(new Vector2d(
                robotCenter.getX() + 48,
                robotCenter.getY() - 60
        ));
        outline.add(new Vector2d(
                robotCenter.getX() + 48,
                robotCenter.getY() - 36
        ));
        //not sure if you have to connect them or not. From the samples, I would assume not
        return outline;
    }

    private List<Vector2d> constructRobotRectFeature(){
        List<Vector2d> outline = new ArrayList<>();
        outline.add(new Vector2d(
                robotCenter.getX() - 59,
                robotCenter.getY() - 35
        ));
        outline.add(new Vector2d(
                robotCenter.getX() - 59,
                robotCenter.getY() + 35
        ));
        outline.add(new Vector2d(
                robotCenter.getX() + 59,
                robotCenter.getY() + 35
        ));
        outline.add(new Vector2d(
                robotCenter.getX() + 59,
                robotCenter.getY() - 35
        ));
        return outline;

    }



}
