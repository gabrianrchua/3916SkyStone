/*
Apex Robotics FTC Team 3916: Main TeleOp for SkyStone season (2019-2020)

Uses a Mecanum-style drivetrain for movement.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp_None_EncoderTest", group="Apex Robotics 3916")
//@Disabled
public class TeleOp_None_EncoderTest extends OpMode {

    private Robot bot = new Robot();

    @Override
    public void init() {
        bot.init(hardwareMap, Robot.DriveType.Mecanum);
    }

    @Override
    public void loop() {
        final double STICK_DEAD_ZONE = 0.1;

        double x = 0;
        double y = 0;
        if (gamepad1.left_bumper) {
            bot.mec_moveAround("leftTurn", 5,0.5);
            telemetry.addData("Status", "Rotating Counterclockwise");
            telemetry.update();
        } else if (gamepad1.right_bumper) {
            bot.mec_moveAround("rightTurn",5,0.5);
            telemetry.addData("Status", "Rotating Clockwise");
            telemetry.update();
        } else if (gamepad1.a) {
            bot.mec_moveAround("forward",10,0.5);
            telemetry.addData("Status", "Going forwards 10 in");
            telemetry.update();
        } else if (gamepad1.b) {
            bot.mec_moveAround("backward",10,0.5);
            telemetry.addData("Status", "Going backwards 10 in");
            telemetry.update();
        } else {
//            if (Math.abs(gamepad1.left_stick_y) > STICK_DEAD_ZONE) {
//                y = gamepad1.left_stick_y;
//            }
//            if (Math.abs(gamepad1.left_stick_x) > STICK_DEAD_ZONE) {
//                x = gamepad1.left_stick_x;
//            }
//            MechPower pwr = bot.mec_drive(x, y);
//            telemetry.addData("Status", "power: x:" + x + " y:" + y + " =pwr:" + pwr.toString());
//            telemetry.update();
            /*double fx = gamepad1.left_stick_x;
            double fy = gamepad1.left_stick_y;
            double sx = gamepad1.right_stick_x;
            double sy = gamepad1.right_stick_y;
            String teleStr = "";
            double slowMult = 0.3;
            if (Math.abs(sx) > STICK_DEAD_ZONE || Math.abs(sy) > STICK_DEAD_ZONE) {
                if (Math.abs(sy) > STICK_DEAD_ZONE) {
                    y = sy * slowMult;
                }
                if (Math.abs(sx) > STICK_DEAD_ZONE) {
                    x = sx * slowMult;
                }
                teleStr = "SLOW";
            } else {
                if (Math.abs(fy) > STICK_DEAD_ZONE) {
                    y = fy;
                }
                if (Math.abs(fx) > STICK_DEAD_ZONE) {
                    x = fx;
                }
                teleStr = "FAST";
            }
            MechPower pwr = bot.mec_drive(x, y);
            telemetry.addData("Drive Stat", teleStr + pwr.toString());
            telemetry.update();*/
        }

    }
}