package org.firstinspires.ftc.teamcode.util;

public class AngleUtils {

    /**
     * Normalize an angle to the range of -PI to +PI
     * @param radians
     * @return
     */
    public static double normalizeRadians(double radians){
        /*
         * We want to normalize radians to the range of -PI to +PI. If temp=(radians+PI)/(2*PI), then
         * we want to normalize that 0 to 1.
         */
        double temp = (radians + Math.PI) / (2.0 * Math.PI);
        temp = temp - Math.floor(temp);                            //This normalized temp to the range of 0 to 1
        return (temp - 0.5) * 2.0 * Math.PI;                       // This normalizes to -PI to +PI
    }
}
