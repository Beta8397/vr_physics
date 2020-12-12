package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "ultimate bot demo", group = "ultimate bot")
public class UltimateBotDemo extends LinearOpMode {

    DcMotor m1, m2, m3, m4;
    BNO055IMU imu;
    DistanceSensor frontDistance, leftDistance, rightDistance, backDistance;
    ColorSensor colorSensor;
//    Servo shooterElevServo;
    Servo shooterTrigServo;
    DcMotor shooterMotor;
    DcMotor intakeMotor;
    DcMotor armMotor;
    Servo handServo;

    public void runOpMode(){
        m1 = hardwareMap.dcMotor.get("back_left_motor");
        m2 = hardwareMap.dcMotor.get("front_left_motor");
        m3 = hardwareMap.dcMotor.get("front_right_motor");
        m4 = hardwareMap.dcMotor.get("back_right_motor");
        m3.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);

        colorSensor = hardwareMap.colorSensor.get("color_sensor");

//        shooterElevServo = hardwareMap.get(Servo.class, "shooter_elev_servo");
        shooterTrigServo = hardwareMap.get(Servo.class, "shooter_trig_servo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");
        intakeMotor.setPower(1);
        shooterMotor.setPower(0.7);

        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        handServo = hardwareMap.get(Servo.class, "hand_servo");




        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        gamepad1.setJoystickDeadzone(0.05f);

        ElapsedTime grabTimer = new ElapsedTime();
        ElapsedTime handTimer = new ElapsedTime();
        float grabPos = 0;

        float handPos = 1;
        handServo.setPosition(handPos);

        waitForStart();
        while (opModeIsActive()){
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = gamepad1.left_trigger - gamepad1.right_trigger;
            if (Math.abs(pa) < 0.05) pa = 0;
            double p1 = -px + py - pa;
            double p2 = px + py + -pa;
            double p3 = -px + py + pa;
            double p4 = px + py + pa;
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);

            if (gamepad1.a) shooterTrigServo.setPosition(1);
            else shooterTrigServo.setPosition(0);

//            double shooterElev = 0.5 * (1 + gamepad1.right_stick_y);
//            shooterElevServo.setPosition(shooterElev);
//            System.out.println("ShooterElev = " + (40.0 - 40.0*shooterElev));

            if (gamepad1.dpad_down && grabTimer.seconds()>0.05){
                grabTimer.reset();
                grabPos = grabPos + 4;
            } else if (gamepad1.dpad_up && grabTimer.seconds()>0.05){
                grabTimer.reset();
                grabPos = grabPos - 4;
            }

            if (gamepad1.dpad_right && handTimer.seconds()>0.05){
                grabTimer.reset();
                handPos = (float)Math.min(1.0, handPos + 0.05);
            } else if (gamepad1.dpad_left && handTimer.seconds()>0.05){
                grabTimer.reset();
                handPos = (float)Math.max(0, handPos - 0.05);
            }

            armMotor.setTargetPosition( (int)(grabPos * 1120.0/360.0));
            armMotor.setPower(1);

            handServo.setPosition(handPos);

            telemetry.addData("GP 1 Lt stick controls fwd/strafe.","");
            telemetry.addData("GP 1 triggers control turn.","");
            telemetry.addData("GP 1 Rt stick controls shooter elev.","");
            telemetry.addData("GP 1 A to shoot", "");

            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
            telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
            telemetry.update();
        }
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}
