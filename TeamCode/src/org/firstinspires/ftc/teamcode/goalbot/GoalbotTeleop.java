package org.firstinspires.ftc.teamcode.goalbot;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mecbot.MecBotTeleOp;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp(name = "GoalBotTeleOp", group = "GoalBot")
public class GoalbotTeleop extends MecBotTeleOp {

    public static final float SHOOTER_DELAYED = 1.0f;

    GoalBot bot= new GoalBot();

    private GoalBot.IntakeState intakeState = GoalBot.IntakeState.OFF;

    ButtonToggle toggleA1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.a;
        }
    };

    ButtonToggle toggleA2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.a;
        }
    };

    ButtonToggle toggleRightBumper2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.right_bumper;
        }
    };

    ButtonToggle toggleLeftBumper2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.left_bumper;
        }
    };

    ButtonToggle toggleB2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.b;
        }
    };

    private boolean grabberClosed = true;
    private boolean shooterOn = false;
    private boolean shooterHigh = false;
    private boolean shooting = false;
    private ElapsedTime shootingTimer= new ElapsedTime();

    public void runOpMode() {
        bot.init(hardwareMap);
        super.setup(bot);

        waitForStart();

        while (opModeIsActive()) {

            handleIntake();

            float armPower = -0.25f * gamepad2.left_stick_y;
            bot.setArmPower(armPower);

            if (toggleA2.update()){
                grabberClosed = !grabberClosed;
                if (grabberClosed) bot.setGrabberClosed();
                else bot.setGrabberOpen();
            }

            if (toggleRightBumper2.update()) {
                shooterOn = !shooterOn;
            }

            if (toggleLeftBumper2.update()) {
                shooterHigh = !shooterHigh;
            }
            if (shooterOn) {
                if (shooterHigh) {
                    bot.setShooterPowerHigh();
                } else {
                    bot.setShooterPowerNormal();
                }
            } else {
                bot.setShooterPower(0);
            }

            float shooterRPM = (float)bot.shooter.getVelocity(AngleUnit.DEGREES) / 6;
            telemetry.addData("shooter_RPM", shooterRPM);

            if (toggleB2.update()) {
                shooting = true;
                shootingTimer.reset();
            } else if(!gamepad2.b) {
                shooting = false;
            }

            if (shooting) {
                if (shootingTimer.seconds() < .5 * SHOOTER_DELAYED) {
                    bot.setKickerEngaged();
                } else if (shootingTimer.seconds() < SHOOTER_DELAYED) {
                    bot.setKickerUnengaged();
                } else {
                    shootingTimer.reset();
                }
            } else {
                bot.setKickerUnengaged();
            }

            doDriveControl();
            telemetry.update();

        }
    }

    private void handleIntake() {

        boolean aToggled = toggleA1.update();
        switch (intakeState) {
            case OFF:
                if (gamepad1.b) {
                    intakeState = GoalBot.IntakeState.REV;
                } else if (aToggled){
                    intakeState = GoalBot.IntakeState.FWD;
                }
                break;
            case FWD:
                if (gamepad1.b) {
                    intakeState = GoalBot.IntakeState.REV;
                } else if (aToggled) {
                    intakeState = GoalBot.IntakeState.OFF;
                }
                break;
            case REV:
                if (!gamepad1.b) {
                    intakeState = GoalBot.IntakeState.OFF;
                }
                break;
        }
        bot.setIntake(intakeState);

    }
}
