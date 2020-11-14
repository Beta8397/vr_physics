package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;

@Autonomous(name = "red auto1", group = "red")
public class RedAutonomous1 extends GoalBotAutonomous {

    public static final float X0 = 9;
    public static final float Y0 = 37;
    public static final float X_SHOOT = 69;
    public static final float Y_SHOOT = 60;
    public static final float angle1 = -174;
    public static final float angle2 = -180;
    public static final float angle3 = -186;

    GoalBot bot = new GoalBot();

    Rings rings = Rings.ZERO;

    public void runOpMode() {
        bot.init(hardwareMap);
        super.setBot(bot);
//        VuforiaNavigator.activate(null, null);
        bot.setPose(X0, Y0, 180);
        bot.setKickerUnengaged();
        waitForStart();
        bot.setShooterPowerNormal();
        rings = getRings();
        driveToPosition(18, 4, X_SHOOT, Y_SHOOT, 180, 2, 1);
        turnToHeading(angle1, 1, 6, 45);
        shoot();
        bot.setIntake(GoalBot.IntakeState.FWD);
//        telemetry.addData("shot 1", "");
//        telemetry.update();
        turnToHeading(angle2,1, 6, 45);
        shoot();
//        telemetry.addData("shot 2", "");
//        telemetry.update();
        turnToHeading(angle3,1, 6, 45);
        shoot();
//        telemetry.addData("shot 3", "");
//        telemetry.update();
        while (opModeIsActive()) {
            continue;
        }

    }
}
