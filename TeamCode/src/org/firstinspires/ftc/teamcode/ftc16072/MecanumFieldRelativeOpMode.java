package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "mecanum field relative opmode", group = "ftc16072")
public class MecanumFieldRelativeOpMode extends OpMode {
    private Robot robot = new Robot();

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        double PY = gamepad1.left_stick_x * -1;
        double PX = gamepad1.left_stick_y * -1;
        double rotate = gamepad1.right_stick_x;

        // mecanumDrive.driveMecanum(forward, strafe, rotate);
        robot.nav.driveFieldRelative(PX, PY, rotate);
    }
}
