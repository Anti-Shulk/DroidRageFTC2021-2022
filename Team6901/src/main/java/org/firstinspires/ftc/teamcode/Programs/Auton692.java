/*
package org.firstinspires.ftc.teamcode.Programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Auton692 extends LinearOpMode {



    public void encoderDrive(double speed, double frontleftInches, double frontrightInches, double backleftInches, double backrightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;
        int newBackLeftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.front_left.getCurrentPosition() + (int) (frontleftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.front_right.getCurrentPosition() + (int) (frontrightInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.back_left.getCurrentPosition() + (int) (backleftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.back_right.getCurrentPosition() + (int) (backrightInches * COUNTS_PER_INCH);

            robot.front_left.setTargetPosition(newFrontLeftTarget);
            robot.front_right.setTargetPosition(newFrontRightTarget);
            robot.back_left.setTargetPosition(newBackLeftTarget);
            robot.back_right.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.front_left.setPower(Math.abs(speed));
            robot.front_right.setPower(Math.abs(speed));
            robot.back_left.setPower(Math.abs(speed));
            robot.back_right.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.front_left.isBusy() && robot.front_right.isBusy())) {
                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.front_left.getCurrentPosition(),
                        robot.front_right.getCurrentPosition());
                telemetry.update();
            }
        }
    }

}
*/