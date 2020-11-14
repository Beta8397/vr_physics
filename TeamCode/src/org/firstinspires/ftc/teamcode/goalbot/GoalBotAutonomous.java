package org.firstinspires.ftc.teamcode.goalbot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mecbot.MecBotAutonomous;

public abstract class GoalBotAutonomous extends MecBotAutonomous {
    enum Rings {
        ZERO, ONE, FOUR
    }

    GoalBot bot;

    public void setBot(GoalBot bot) {
        this.bot = bot;
        super.setBot(bot);

    }

    public Rings getRings() {
        //TODO: CODE THE RING DETECTION
        return Rings.FOUR;
    }

    public void shoot() {
        bot.setKickerEngaged();
        sleep(500);
        bot.setKickerUnengaged();
    }

}
