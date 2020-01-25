/*
Apex Robotics FTC Team 3916: Main TeleOp for SkyStone season (2019-2020)

Uses a Mechanum-style drivetrain for movement.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Slow Drive", group="Apex Robotics 3916")
//@Disabled
public class SlowDriveTeleOp extends OpMode {

    private Robot bot = new Robot();

    @Override
    public void init() {
        bot.init(hardwareMap, Robot.DriveType.Mechanum);
    }

    @Override
    public void loop() {
        final double STICK_DEAD_ZONE = 0.1;

        double x = 0;
        double y = 0;
        //John Switched the values of bot.mech_rotate(int)
        if (gamepad1.left_bumper) {
            bot.mech_rotate(0);
            telemetry.addData("Status", "Rotating Counterclockwise");
            telemetry.update();
        } else if (gamepad1.right_bumper) {
            bot.mech_rotate(1);
            telemetry.addData("Status", "Rotating Clockwise");
            telemetry.update();
        } else {
            double sx = gamepad1.left_stick_x;
            double sy = gamepad1.left_stick_y;
            double slowMult = 0.3;
            if (Math.abs(sy) > STICK_DEAD_ZONE) {
                y = sy * slowMult;
            }
            if (Math.abs(sx) > STICK_DEAD_ZONE) {
                x = sx * slowMult;
            }
            MechPower pwr = bot.mech_drive(x, y);
            telemetry.addData("Drive Stat", pwr.toString());
            telemetry.update();
        }
        String message = "Wack nothing's happening...";
        //stage 1 lift
        if (Math.abs(gamepad2.left_stick_y) > STICK_DEAD_ZONE) {
            bot.aux_claw(gamepad2.left_stick_y);
            message = "STAGE 1 LIFTING POWER " + gamepad2.left_stick_y;
        } else {
            bot.aux_claw(0);
        }
        //stage 2 lift
        if (gamepad2.dpad_up) {
            bot.aux_claw2(1);
            message = "STAGE 2 LIFT POWER 1";
        } else if (gamepad2.dpad_down) {
            bot.aux_claw2(-1);
            message = "STAGE 2 LIFT POWER -1";
        } else {
            bot.aux_claw2(0);
        }
        telemetry.addData("Lift", message);
        //claw
        if (Math.abs(gamepad2.right_stick_y) > STICK_DEAD_ZONE) {
            bot.aux_claw3(gamepad2.right_stick_y);
            message = "CLAW LIFT POWER " + gamepad2.right_stick_y;
        } else {
            bot.aux_claw3(0);
            message = "CLAW LIFT POWER 0";
        }
        telemetry.addData("Claw", message);
    }
}