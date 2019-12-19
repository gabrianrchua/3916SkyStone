/*
Apex Robotics FTC Team 3916: Main TeleOp for SkyStone season (2019-2020)

Uses a Mechanum-style drivetrain for movement.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Blinear Slide Boi", group="Apex Robotics 3916")
//@Disabled
public class LinearSlideTeleOp extends OpMode {

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
        double rx = 0;
        if (Math.abs(gamepad1.left_stick_y) > STICK_DEAD_ZONE) {
            y = gamepad1.left_stick_y;
        }
        if (Math.abs(gamepad1.left_stick_x) > STICK_DEAD_ZONE) {
            x = gamepad1.left_stick_x;
        }
        if (Math.abs(gamepad1.right_stick_x) > STICK_DEAD_ZONE) {
            rx = gamepad1.right_stick_x;
        }
        if (Math.abs(gamepad2.left_stick_y) > STICK_DEAD_ZONE) {
            bot.aux_lift(gamepad2.left_stick_y);
        }
        if (Math.abs(gamepad2.right_stick_y) > STICK_DEAD_ZONE) {
            bot.aux_claw(gamepad2.right_stick_y);
        }
        MechPower pwr = bot.mech_drive(x, y, rx);
        telemetry.addData("Powers", "x:" + x + " y:" + y + " =pwr:" + pwr.toString());
        telemetry.update();
    }
}