package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.mecbot.MecBotAutonomous;
import org.firstinspires.ftc.teamcode.util.CubicSpline2D;

@Autonomous(name = "example auto", group = "test")
public class ExampleAuto extends MecBotAutonomous {

    MecBot bot = new MecBot();

    DcMotor armMotor;
    Servo handServo;
    DcMotor shooterMotor;
    Servo triggerServo;
    Servo shooterElevServo;

    float[] points1 = new float[]{30, 9, 12, 72, 48, 130};
    CubicSpline2D spline1 = new CubicSpline2D(points1, 90, 0);

    float[] points2 = new float[]{48, 130, 29, 96, 29, 76};
    CubicSpline2D spline2 = new CubicSpline2D(points2, 180, -90);

    public void runOpMode(){
        bot.init(hardwareMap);
        super.setBot(bot);

        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        handServo = hardwareMap.get(Servo.class, "hand_servo");
        triggerServo = hardwareMap.get(Servo.class, "shooter_trig_servo");
        shooterElevServo = hardwareMap.get(Servo.class, "shooter_elev_servo");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");

        handServo.setPosition(1);
        sleep(300);
        armMotor.setTargetPosition(420);
        armMotor.setPower(1);
        shooterMotor.setPower(0.8);

        bot.setPose(30, 9, 90);

        waitForStart();

        driveSpline(50, false, spline1);

        armMotor.setTargetPosition(168);
        armMotor.setPower(1);
        sleep(300);
        handServo.setPosition(0);
        sleep(50);
        armMotor.setTargetPosition(448);
        armMotor.setPower(1);

        driveSpline(50, true, spline2);

//        System.out.println("Wait Time: " + waitUntilResting(0.2f, 0.2f, 50));

        turnToHeading(90, 0.5f, 4, 30);

        shooterElevServo.setPosition(0.15);
        sleep(300);
        triggerServo.setPosition(1);
        sleep(300);

        triggerServo.setPosition(0);
        sleep(300);
        triggerServo.setPosition(1);
        sleep(300);

        triggerServo.setPosition(0);
        sleep(300);
        triggerServo.setPosition(1);
        sleep(1000);

        bot.updateOdometry();
        System.out.println("Heading: " + Math.toDegrees(bot.getPose().theta));
    }

}
