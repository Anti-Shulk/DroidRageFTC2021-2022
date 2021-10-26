/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.main.beta;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class LinearOpModeBeta extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Drivetrain
    private DcMotorEx leftFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear = null;

    // Cycles variable (to calculate frequency)
    private int cycles = 0;

    // Slow mode variable for slow mode
    double slowMode = 1;

    // Arm Motor
    private DcMotor armMotor = null;

    // Duck Motor
    private DcMotorEx duckMotor = null;

    // Servos

    private Servo rightClaw = null;
    private Servo leftClaw = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Drivetrain Initialization
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront  = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Arm Motor Initialization
        armMotor = hardwareMap.get(DcMotor.class,"armMotor  ");

        // Duck Motor

        duckMotor = hardwareMap.get(DcMotorEx.class,"duckMotor");

        // Servo Claw Initialization

        leftClaw = hardwareMap.get(Servo.class,"leftClaw");
        rightClaw = hardwareMap.get(Servo.class,"rightClaw");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Drivetrain Motor Directions
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);

        // Arm Motor Directions
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Duck Motor Directions
        duckMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Servo Directions
        leftClaw.setDirection(Servo.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.FORWARD);

        // Adjusting the Zero Power Behavior changes how the motors behaved when a
        // Power of 0 is applied.

        // Drivetrain Zero Power Behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm Motor Zero Power Behavior
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Duck Motor Zero Power Behavior

        duckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turn on Run Using Encoder to use the built in PID controller

        // Drivetrain Encoders
        /*
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Reset Encoders before starting OpMode
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */

        // Tell the driver (by printing a message on the driver station) that initialization is complete.
        telemetry.addData("Status", "Initialized");

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        }

    }
    private void move (String direction, long distance) {
        double power = 0.5;
        if (direction.equals("forward")) {
            leftFront.setPower(power);
            rightFront.setPower(power);
            leftRear.setPower(power);
            rightRear.setPower(power);
        }
        if (direction.equals("backward")) {
            leftFront.setPower(-power);
            rightFront.setPower(-power);
            leftRear.setPower(-power);
            rightRear.setPower(-power);
        }
        if (direction.equals("left")) {
            leftFront.setPower(-power);
            rightFront.setPower(power);
            leftRear.setPower(-power);
            rightRear.setPower(power);
        }
        if (direction.equals("right")) {
            leftFront.setPower(power);
            rightFront.setPower(-power);
            leftRear.setPower(power);
            rightRear.setPower(-power);
        }
        sleep(distance);
    }
    private void move (String direction, long distance, double power) {
        if (direction.equals("forward")) {
            leftFront.setPower(power);
            rightFront.setPower(power);
            leftRear.setPower(power);
            rightRear.setPower(power);
        }
        if (direction.equals("backward")) {
            leftFront.setPower(-power);
            rightFront.setPower(-power);
            leftRear.setPower(-power);
            rightRear.setPower(-power);
        }
        if (direction.equals("left")) {
            leftFront.setPower(-power);
            rightFront.setPower(power);
            leftRear.setPower(-power);
            rightRear.setPower(power);
        }
        if (direction.equals("right")) {
            leftFront.setPower(power);
            rightFront.setPower(-power);
            leftRear.setPower(power);
            rightRear.setPower(-power);
        }
        sleep(distance);
    }
    }