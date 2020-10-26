package org.firstinspires.ftc.teamcode.util.geometry;

import android.graphics.PointF;

/**
 * Created by FTC Team 8397 on 12/18/2018.
 */
public class Circle {

    public enum Direction {CLOCK, COUNTERCLOCK };

    public PointF center;

    public float radius;

    public Circle(PointF center, float radius){
        this.center = center;
        this.radius = radius;
    }

    public Circle(float xc, float yc, float radius){
        this.center = new PointF(xc, yc);
        this.radius = radius;
    }

    public static Circle fromArc( PointF point0, PointF point1, float arcHeight, Direction direction){

        float chordCenterX = 0.5f * (point1.x + point0.x);
        float chordCenterY = 0.5f * (point1.y + point0.y);

        float xDiff = point1.x - point0.x;
        float yDiff = point1.y - point0.y;

        float chordLength = (float)Math.sqrt( xDiff * xDiff + yDiff * yDiff );

        float radius = arcHeight / 2.0f + chordLength * chordLength / (8.0f * arcHeight);
        float xc, yc;

        if (direction == Direction.COUNTERCLOCK){
            xc = chordCenterX - yDiff * (radius - arcHeight) / chordLength;
            yc = chordCenterY + xDiff * (radius - arcHeight) / chordLength;
        } else {
            xc = chordCenterX + yDiff * (radius - arcHeight) / chordLength;
            yc = chordCenterY - xDiff * (radius - arcHeight) / chordLength;
        }

        return new Circle(xc, yc, radius);
    }
}
