package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.mecbot.MecBotAutonomous;
import org.firstinspires.ftc.teamcode.util.CubicSpline2D;

@Autonomous(name = "DriveSpline", group="MechBot")
public class DriveSpline extends MecBotAutonomous {

    //public MecBot bot = new MecBot(MechBot.MotorType.NeverestOrbital20, 31.92, 35.6, 40.6);
    MecBot bot = new MecBot();

    public void runOpMode(){
        bot.init(hardwareMap);
        super.setBot(bot);

        /**
         * Create the cubic spline by supplying an array of points (x0, y0, x1, y1, x2, y2, ...) and a
         * beginning and ending travel direction (in degrees).
         *
         * Field coordinates: (0,0) is center of field. X-axis is right; Y-axis is up. Each square is 61 cm wide.
         *
         * Start robot at lower left intersection of squares (-122, -122), facing up (heading = 90 degrees)
         */

        float[] points = new float[]{-48, -48, -24, 24, 0, 0, 24, -24, 48, 48};
        //float[] points = new float[]{-36, -36, -36, 36, 36, 36, 36, -36, -36, -36};
        CubicSpline2D spline = new CubicSpline2D(points, 90, 90);

        waitForStart();

        /**
         * Must reset odometry to indicate starting robot position and heading.
         */
        bot.setPose(-48, -48, 90);
        //bot.setPose(-36, -36, 135);

        driveSpline(24, false, spline);
    }
}
