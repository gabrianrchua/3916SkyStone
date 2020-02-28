package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Base Plate RED Autonomous", group = "Apex Robotics 3916")
public class RedComplexAuto extends OpMode {
    private Robot bot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    private String timeOfCompletion;

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
        switch (state) {
            // even numbers for performing an action while odd values for state are used for waiting, seen in default case
            case 0:
                // drive backwards
                new Thread(new Tasker("drive,0,1,1100")).start();
                state++;
                break;
            case 2:
                // bring down base grabby claw
                new Thread(new Tasker("BP claw down,700")).start(); // command, pause
                state++;
                break;
            case 4:
                // drive forward
                new Thread(new Tasker("drive,0,-1,1100")).start();
                state++;
                break;
            case 6:
                // turn right
                new Thread(new Tasker("slow turn,1,0.35,2800")).start();
                state++;
                break;
            case 8:
                // wait
                new Thread(new Tasker("pause,500")).start();
                state++;
                break;
            case 10:
                // turn other way for a second
                new Thread(new Tasker("slow turn,0,0.35,300")).start();
                state++;
                break;
            case 12:
                // wait
                new Thread(new Tasker("pause,501")).start();
                state++;
                break;
            case 14:
                // bring up base grabby claw
                //new Thread(new Tasker("BP claw up,750")).start();
                new Thread(new Tasker("BP claw up,1200")).start();
                state++;
                break;
            case 16:
                // drive backwards
                new Thread(new Tasker("drive,0,-1,700")).start();
                state++;
                break;
            default:
                //pause for a bit
                //new Thread(new Tasker("pause,1000")).start();
                //state++;
                break;
        }
        //new Thread(new Tasker("pause,1000")).start();
        telemetry.addData("runtime", runtime.toString() + " seconds");

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
                    bot.mech_drive(x, y);
                    pause(Long.parseLong(split[3]));
                    bot.stopDriving();
                    telemetryMsg = "done driving";
                    break;
                case "turn":
                    // rotating the mech drive as in if a controller input
                    int direction = Integer.parseInt(split[1]);
                    telemetryMsg = "turning with direction " + direction;
                    bot.mech_rotate(direction);
                    pause(Long.parseLong(split[2]));
                    telemetryMsg = "done rotating";
                    break;
                case "slow turn":
                    // turn slowly than turn
                    int directionn = Integer.parseInt(split[1]);
                    telemetryMsg = "turning slowly with direction " + directionn;
                    bot.mech_rotate(directionn, Double.parseDouble(split[2]));
                    pause(Long.parseLong(split[3]));
                    telemetryMsg = "done turning slowly";
                    break;
                case "pause":
                    //wait for a bit
                    telemetryMsg = "waiting for a bit";
                    pause(Long.parseLong(split[1]));
                    telemetryMsg = "done waiting";
                    break;
                case "open claw":
                    // Open the claw
                    telemetryMsg = "Opening the claw";
                    bot.aux_claw3_direct(1);
                    pause(Long.parseLong(split[2]));
                    //bot.aux_claw3(0);
                    telemetryMsg = "Claw is open";
                    break;
                case "close claw":
                    // Close the claw
                    telemetryMsg = "Closing the claw";
                    bot.aux_claw3_direct(0);
                    pause(Long.parseLong(split[2]));
                    //bot.aux_claw3(0);
                    telemetryMsg = "Claw is closed";
                    break;
                case "BP claw up":
                    bot.aux_claw4(1);
                    pause(Long.parseLong(split[1]));
                    bot.aux_claw4(0);
                    break;
                case "BP claw down":
                    bot.aux_claw4(-1);
                    pause(Long.parseLong(split[1]));
                    bot.aux_claw4(0);
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