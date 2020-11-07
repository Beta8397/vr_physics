package org.firstinspires.ftc.teamcode.util;
/**
 * Represents the linear speed (x,y) and angular speed (in radians/sec) of the robot. vx and vy could be either in
 * field coordinates or world coordinates.
 * NOTE:  Speed is immutable.
 */
public class Speed {
    public final float vx;
    public final float vy;
    public final float vTheta;

    public Speed(float vxx, float vyy, float vvTheta){
        vx = vxx;
        vy = vyy;
        vTheta = vvTheta;
    }
}
