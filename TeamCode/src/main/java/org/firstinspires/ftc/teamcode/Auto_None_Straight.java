package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Straight Ahead Autonomous", group = "Apex Robotics 3916")
public class Auto_None_Straight extends OpMode {
    private Robot bot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    private String timeOfCompletion;

    //This (codeState) variable basically holds whether or not the code has run.
    // If it has (and there are no other states to run), then don't do anything.
    private int codeState = 0;

    //This (state) variable is for the old switch statement code but idk
    private int state = 0;
    private String telemetryMsg;

    @Override
    public void init() {
        bot.init(hardwareMap, Robot.DriveType.Mecanum);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void stop() {
        bot.stopDriving();
    }

    @Override
    public void loop() {
        if (codeState == 0){
            //Enter code here

            //pause for a bit
            new Thread(new Tasker("pause,1000"));

            // drive forward
            new Thread(new Tasker("drive,0,0.5,1800")).start();

            //pause for a bit
            new Thread(new Tasker("pause,1000"));

            // autonomous is done as robot has parked
            telemetryMsg = "Autonomous completed in " + runtime.toString() + " seconds";

            codeState = 1;
        }
        else {
            //If the code is done, just wait I guess
            new Thread(new Tasker("pause,1000"));
        }


        telemetry.addData("Runtime:", runtime.toString() + " seconds");

        if (telemetryMsg != null) {
            telemetry.addData("*", "-----------");
            telemetry.addData("*", telemetryMsg);
        }

        telemetry.update();
    }

    private class Tasker implements Runnable {
        private String task;
        private String orientation;

        Tasker(String task) {
            this.task = task;
        }

        @Override
        public void run() {
            // first value from split string should be the task to perform
            // other split values are arguments
            String[] split = task.split(",");

            switch (split[0]) {
                case "drive":
                    // drive with args for x and y as in if a controller input
                    double x = Double.parseDouble(split[1]);
                    double y = Double.parseDouble(split[2]);
                    telemetryMsg = "driving at x=" + x + " y=" + y;
                    bot.mec_drive(x, y);
                    pause(Long.parseLong(split[3]));
                    bot.stopDriving();
                    telemetryMsg = "done driving";
                    break;
                case "turn":
                    // rotating the mec drive as in if a controller input
                    int direction = Integer.parseInt(split[1]);
                    telemetryMsg = "turning with direction " + direction;
                    bot.mec_rotate(direction);
                    pause(Long.parseLong(split[2]));
                    telemetryMsg = "done rotating";
                    break;
                case "pause":
                    //wait for a bit
                    telemetryMsg = "waiting for a bit";
                    pause(Long.parseLong(split[1]));
                    telemetryMsg = "done waiting";
                    break;
                default:
                    telemetryMsg = split[0] + " is either spelled wrong or hasn't been implemented yet";
                    break;
            }

            state++;
        }
        private void pause(long ms) {
            try {
                Thread.sleep(ms);
            } catch (InterruptedException ex) {
                telemetry.addData("ERROR", ex.getMessage());
            }
        }
    }
}