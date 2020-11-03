package org.firstinspires.ftc.teamcode.mecbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.mecbot.MecBotTeleOp;

@TeleOp(name = "test teleop", group = "test")
public class TestTeleOp extends MecBotTeleOp {

    MecBot bot = new MecBot();

    public void runOpMode(){
        bot.init(hardwareMap);
        super.setBot(bot);
        bot.setPose(0, 0, 0);

        waitForStart();

        while(opModeIsActive()){
            bot.updateOdometry();
            telemetry.addData("POS", "X = %.1f  Y = %.1f  H = %.1f", bot.getPose().x,
                    bot.getPose().y, Math.toDegrees(bot.getPose().theta));
            doDriveControl();
            telemetry.update();
        }
    }

}
