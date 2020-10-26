package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.mecbot.MecBotAutonomous;
import org.firstinspires.ftc.teamcode.util.ParametricFunction2D;

@Autonomous(name="DriveLissajous", group="MechBot")
public class DriveLissajous extends MecBotAutonomous {
    //public MecBot bot = new MechBot(MechBot.MotorType.NeverestOrbital20, 32.75, 31.75, 31.75);
    MecBot bot = new MecBot();

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        super.setBot(bot);
        waitForStart();
        bot.setPose(40, 0, 90);
        driveFunction(40, 0, new ParametricFunction2D() {
            float kx = 5f;
            float ky = 4f;
            @Override
            public VectorF p(float s) {
                return new VectorF(40*(float)Math.cos(kx*s), 40*(float)Math.sin(ky*s));
            }

            @Override
            public VectorF d1(float s) {
                return new VectorF(-40*kx*(float)Math.sin(kx*s),  40*ky*(float)Math.cos(ky*s));
            }

            @Override
            public VectorF d2(float s) {
                return new VectorF(-40*kx*kx*(float)Math.cos(kx*s), -40*ky*ky*(float)Math.sin(ky*s));
            }
        }, new Predicate() {
            @Override
            public boolean isTrue() {
                return false;
            }
        });
    }
}
