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

package org.firstinspires.ftc.teamcode.luckyDrivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="10862TeleOp", group="Linear Opmode")
public class Gamepad_BasicOpMode_Linear10862_practicedrivetrain extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Declare OpMode members.
    //null = no value
    private DcMotorEx leftFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear = null;
    private DcMotor carouselMotor = null;
    private DcMotor otherMotor = null;
    //Servos
    private Servo rightServo = null;
    private Servo leftServo = null;

    @Override
    @SuppressWarnings("FieldCanBeLocal")

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront  = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");
        carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");
        otherMotor = hardwareMap.get(DcMotor.class, "otherMotor");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Drivetrain Motor Directions
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);
        carouselMotor.setDirection(DcMotorEx.Direction.REVERSE);
        otherMotor.setDirection(DcMotor.Direction.FORWARD);
        //Servo Directions
        rightServo.setDirection(Servo.Direction.FORWARD);
        leftServo.setDirection(Servo.Direction.FORWARD);

        // Adjusting the Zero Power Behavior changes how the motors behaved when a
        // Power of 0 is applied.

        //Drivetrain Zero Power Behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        otherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Do servos have somthing similiar to ZeroPowerBehavior?

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            //double = 2 digits after the decimal
            double leftPower;
            double rightPower;

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            //Drivetrain
            double drive = -gamepad1.left_stick_y;
            double turn  =  -gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            //Wouldn't the right power also be drive+turn?

            // Send calculated power to wheels
            leftFront.setPower(leftPower * 0.6);
            leftRear.setPower(leftPower * 0.6);
            rightFront.setPower(rightPower * 0.6);
            rightRear.setPower(rightPower * 0.6);

            //CarouselMotor
            if (gamepad1.right_bumper) {
                carouselMotor.setPower(0.4);
            } else {
                carouselMotor.setPower(0);
            }

            //100 Milliseconds = 1 Second
            //Servo

                if (gamepad2.a) {
                rightServo.setPosition(0.3);
                leftServo.setPosition(0.3);
            }
                if (gamepad2.b) {
                    rightServo.setPosition(0.1);
                    leftServo.setPosition(0.1);
                }
                if (gamepad2.y) {
                    rightServo.setPosition(0.2);
                    leftServo.setPosition(0.2);
                    sleep(100);
                }
            }



            /* && means AND
            || means OR */

            //otherMotor (Intake/Outtake)
            if (gamepad2.right_bumper) {
                otherMotor.setPower(0.65);
            }
            if (gamepad2.left_bumper) {
                otherMotor.setPower(-0.3);
            }

            // ! means not
            if (!gamepad2.left_bumper && !gamepad1.right_bumper){
                otherMotor.setPower(0);
            }




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
        }
    }

