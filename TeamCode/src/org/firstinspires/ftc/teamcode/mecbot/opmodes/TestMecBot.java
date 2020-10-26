package org.firstinspires.ftc.teamcode.mecbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;

@TeleOp(name="Test Mech Bot", group="tutorial")
public class TestMecBot extends LinearOpMode {
    /*
     * The MechBot object that will be used by this op mode.
     */
    MecBot bot;
    public void runOpMode() {
        bot = new MecBot();    //Obtain a new instance of MechBot and assign it to bot.
        bot.init(hardwareMap);  //Initialize the bot
        gamepad1.setJoystickDeadzone(0.05f);
        waitForStart();

        while(opModeIsActive()) {
            /*
             * Update our estimate of the robot's position on the field, then display it with telemetry.
             */
            bot.updateOdometry();
            telemetry.addData("X = ", bot.getPose().x);
            telemetry.addData("Y = ", bot.getPose().y);
            telemetry.addData("Theta = ", Math.toDegrees(bot.getPose().theta));
            telemetry.update();

            /*
             * Set drive power using the gamepad
             */
            bot.setDrivePower(gamepad1.left_stick_x,-gamepad1.left_stick_y,
                    -gamepad1.right_stick_x);
        }
    }
}
