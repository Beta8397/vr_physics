package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;

public class GoalBot extends MecBot {

    public static final float GRABBER_OPEN_POSITION = 0f;
    public static final float GRABBER_CLOSED_POSITION = 1f;
    public static final float SHOOTER_POWER_NORMAL = 0.8f;
    public static final float SHOOTER_POWER_HIGH = 1.0f;
    public static final float KICKER_ENGAGED = 1;
    public static final float KICKER_UNENGAGED = 0;

    DcMotorEx intakeMotor;
    DcMotorEx armMotor;
    DcMotorEx shooter;
    Servo kicker;
    Servo grabber;
    enum IntakeState {
        OFF, FWD, REV
    }

    public GoalBot(){
        super();
    }


    public void init(HardwareMap hwMap){
        super.init(hwMap);
        intakeMotor = hwMap.get(DcMotorEx.class, "intake_motor");
        armMotor = hwMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        grabber = hwMap.get(Servo.class, "hand_servo");
        shooter = hwMap.get(DcMotorEx.class, "shooter_motor");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        kicker = hwMap.get(Servo.class, "shooter_trig_servo");
    }

    public void setIntake(IntakeState intakeState) {
        if (intakeState == IntakeState.OFF) {
            setIntakePower(0);
        } else if (intakeState == IntakeState.FWD) {
            setIntakePower(1);
        } else {
            setIntakePower(-1);
        }
    }

    public void setShooterPower(float pwr) {
        shooter.setPower(pwr);
    }

    public void setShooterPowerNormal() {
        setShooterPower(SHOOTER_POWER_NORMAL);
    }

    public void setShooterPowerHigh() {
        setShooterPower(SHOOTER_POWER_HIGH);
    }

    public void setKickerPosition(float pos) {
        kicker.setPosition(pos);
    }

    public void setKickerEngaged() {
        setKickerPosition(KICKER_ENGAGED);
    }

    public void setKickerUnengaged() {
        setKickerPosition(KICKER_UNENGAGED);
    }

    public void setArmPower(float pwr){
        armMotor.setPower(pwr);
    }

    public void setGrabberPosition(float pos){
        grabber.setPosition(pos);
    }

    public void setGrabberOpen(){
        grabber.setPosition(GRABBER_OPEN_POSITION);
    }

    public void setGrabberClosed(){
        grabber.setPosition(GRABBER_CLOSED_POSITION);
    }

    public void setIntakePower(float p){
        intakeMotor.setPower(p);
    }
}
