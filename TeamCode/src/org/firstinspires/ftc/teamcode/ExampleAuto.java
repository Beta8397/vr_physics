package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    float[] points1 = new float[]{9, 37, 48, 60, 72, 36};
    CubicSpline2D spline1 = new CubicSpline2D(points1, 0, 0);



    public void runOpMode(){
        bot.init(hardwareMap);
        super.setBot(bot);

        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        handServo = hardwareMap.get(Servo.class, "hand_servo");
        triggerServo = hardwareMap.get(Servo.class, "shooter_trig_servo");
        shooterElevServo = hardwareMap.get(Servo.class, "shooter_elev_servo");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");

        handServo.setPosition(1);
        sleep(300);
        armMotor.setTargetPosition(0);
        armMotor.setPower(1);
        shooterMotor.setPower(0.8);

        bot.setPose(9, 37, 180);

        waitForStart();

        driveSpline(50, true, spline1); shooterElevServo.setPosition(0.15);

        turnToHeading(180, 0.5f, 12, 90);

        triggerServo.setPosition(1);
        sleep(300);

        triggerServo.setPosition(0);
        sleep(300);
        triggerServo.setPosition(1);
        sleep(300);

        triggerServo.setPosition(0);
        sleep(300);
        triggerServo.setPosition(1);
        sleep(300);
        triggerServo.setPosition(0);

        while (opModeIsActive()) continue;
    }

}
