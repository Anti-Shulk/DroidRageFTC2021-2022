package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Teleop(name="Teleop18458", group="Linear Opmode")
class TeleOp18458 extends LinearOpMode {

    // Uses hardware file. Replace "teamNumber" with the Team number for the
    // bot. To use the hardware file, type robot.[device name] instead of just
    // [device name]
    Hardware_18458 robot   = new Hardware_18458();   // This uses the hardware class

    private final ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Add telementry data
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (until driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable to control slow mode
            double slowMode;

            // The slow mode variable, when multipled by any value,
            // will allow for slower movement for that value

            if (gamepad1.left_bumper) {
                slowMode = 0.5;
            } else {
                slowMode = 1;
            }

            double StikPower = gamepad2.left_stick_x;
            double twist = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double drive = gamepad1.left_stick_y;

            double[] speeds = {
                    (-(twist * slowMode) - (strafe * slowMode) - (drive * slowMode)),
                    (-(twist * slowMode) - (strafe * slowMode) + (drive * slowMode)),
                    (-(twist * slowMode) + (strafe * slowMode) - (drive * slowMode)),
                    (-(twist * slowMode) + (strafe * slowMode) + (drive * slowMode))
            };
            double max = Math.abs(speeds[0]);
            for (double speed : speeds) {
                if (max < Math.abs(speed)) max = Math.abs(speed);
            }

            // If and only if the maximum is outside of the range we want it to be,
            // normalize all the other speeds based on the given speed value.
            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }

            // apply the calculated values to the motors.
            robot.FL.setPower(speeds[0]);
            robot.FR.setPower(-speeds[1]);
            robot.BL.setPower(speeds[2]);
            robot.BR.setPower(-speeds[3]);
            }
        }
    }
