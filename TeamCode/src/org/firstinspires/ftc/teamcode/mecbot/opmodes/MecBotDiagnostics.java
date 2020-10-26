/**
 * Diagnostic OpMode for use with our 2nd MechBot (with Neverest 40s, gear ratio 0.5, and Rev Hub mounted on its side
 */

package org.firstinspires.ftc.teamcode.mecbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;

@TeleOp(name = "MecBotDiagnostics", group = "Test")
//@Disabled
public class MecBotDiagnostics extends LinearOpMode {

//    private MechBot bot = new MechBot(MechBot.MotorType.NeverestOrbital20, 32.75, 31.75, 31.75,
//            BNO055Enhanced.AxesMap.XZY, BNO055Enhanced.AxesSign.PNP, 1);

    private MecBot bot = new MecBot();

    @Override
    public void runOpMode() {

        bot.init(hardwareMap);

        bot.setHeadingDegrees(0);

        waitForStart();

        while (opModeIsActive()){
            doOneIteration();
            telemetry.update();
        }



    }

    protected void doOneIteration(){

        bot.updateOdometry();

        if (gamepad1.a) handleMotorPowers(0.2f,0,0,0);
        else if (gamepad1.x) handleMotorPowers(0,0.2f,0,0);
        else if (gamepad1.y) handleMotorPowers(0,0,0.2f,0);
        else if (gamepad1.b) handleMotorPowers(0,0,0,0.2f);
        else if (gamepad1.dpad_right) handleBotPowers(0.2f, 0, 0);
        else if (gamepad1.dpad_up) handleBotPowers(0, 0.2f, 0);
        else if (gamepad1.dpad_down) handleBotPowers(0, 0, 0.2f);
        else {
            float px = gamepad1.left_stick_x;
            float py = -gamepad1.left_stick_y;
            float pa = -gamepad1.right_stick_x;
            if (Math.abs(px) < 0.05) px = 0;
            if (Math.abs(py) < 0.05) py = 0;
            if (Math.abs(pa) < 0.05) pa = 0;
            handleBotPowers(px, py, pa);
        }

        telemetry.addData("Heading", "%.1f Degrees", bot.getPose().theta * 180.0 / Math.PI);
        telemetry.addData("Odometry", "X = %.1f  Y = %.1f  Theta = %.1f",
                bot.getPose().x, bot.getPose().y, bot.getPose().theta *180.0 / Math.PI);
        telemetry.addData("Encoders", "BL %d  FL %d  FR %d  BR %d",
                bot.getBackLeft().getCurrentPosition(), bot.getFrontLeft().getCurrentPosition(),
                bot.getFrontRight().getCurrentPosition(), bot.getBackRight().getCurrentPosition());



    }

    private void handleBotPowers(float px, float py, float pa){
        bot.setDrivePower(px, py, pa);
        telemetry.addData("Robot Power", "PX = %.2f  PY = %.2f  PA = %.2f", px, py, pa);
    }

    private void handleMotorPowers(float p1, float p2, float p3, float p4){
        bot.getBackLeft().setPower(p1);
        bot.getFrontLeft().setPower(p2);
        bot.getFrontRight().setPower(p3);
        bot.getBackRight().setPower(p4);
        telemetry.addData("Motor Powers", "BL: %.2f  FL: %.2f  FR: %.2f  BR: %.2f", p1, p2, p3, p4);
    }

}
