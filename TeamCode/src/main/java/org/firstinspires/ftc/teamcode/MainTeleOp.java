/*
Apex Robotics FTC Team 3916: Tank Drive Template
Created with Android Studio v3.3.1, Gradle v4.10.1
Template created by Gabrian Chua (2019)

This TeleOp class defines a template for a basic Two Motor Tank Drive setup.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Main TeleOp", group="Apex Robotics 3916")
//@Disabled
public class MainTeleOp extends OpMode {

    private Robot bot = new Robot();

    @Override
    public void init() {
        bot.init(hardwareMap, Robot.DriveType.Mechanum);
    }

    @Override
    public void loop() {
        final double STICK_DEAD_ZONE = 0.1;
        double MOTOR_POWER = 1;

        double x = 0;
        double y = 0;
        if (Math.abs(gamepad1.left_stick_y) > STICK_DEAD_ZONE) {
            y = gamepad1.left_stick_y;
        }
        if (Math.abs(gamepad1.left_stick_x) > STICK_DEAD_ZONE) {
            x = gamepad1.left_stick_x;
        }
        MechPower pwr = bot.mech_drive(x, y);
        telemetry.addData("Powers", "x:" + x + " y:" + y + " =pwr:" + pwr.toString());
        telemetry.update();
    }
}