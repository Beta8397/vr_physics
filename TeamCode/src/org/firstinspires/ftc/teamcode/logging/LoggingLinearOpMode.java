package org.firstinspires.ftc.teamcode.logging;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 *
 * Abstrct class which extends LinearOpMode, adding simple logging capability.
 */

public abstract class LoggingLinearOpMode extends LinearOpMode {

    /**
     * Implementation of runOpMode: initializes logging, passes control to runLoggingOpmode, then
     * makes sure that BetaLog gets closed. Made final so it can't be overridden by subclasses.
     */
    @Override
    public final void runOpMode() throws InterruptedException{
        try {
            BetaLog.initialize();
            runLoggingOpmode();
        }
        finally{
            BetaLog.close();
        }
    }


    /**
     * Subclasses will need to implement runLoggingOpmode(). It will have the same function as
     * runOpMode does in traditional LinearOpModes.
     */
    public abstract void runLoggingOpmode() throws InterruptedException;



}
