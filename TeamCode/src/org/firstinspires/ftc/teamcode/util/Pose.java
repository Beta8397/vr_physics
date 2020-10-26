package org.firstinspires.ftc.teamcode.util;
/**
 * Represents the position (x,y) and heading (in radians) of the robot on the field.
 * NOTE:  Pose is immutable.
 */
public class Pose {
    public final float x;
    public final float y;
    public final float theta;

    public Pose(float xx, float yy, float ttheta){
        x = xx;
        y = yy;
        theta = ttheta;
    }
}
