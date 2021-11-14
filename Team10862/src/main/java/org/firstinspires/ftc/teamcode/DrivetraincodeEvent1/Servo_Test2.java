package org.firstinspires.ftc.teamcode.DrivetraincodeEvent1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;

@TeleOp(name="ServoTest2")
public class Servo_Test2 extends OpMode {
    //hardware initialization stuff
     Servo servo;
    double pos = 0;
//rightServo - starting position 0.25
    //leftServo - starting position 0.8
    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        servo= hardwareMap.get(Servo.class, "leftServo");
        servo.setDirection(Servo.Direction.FORWARD);
    }
//rightServo B is down
    //leftServo
    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        if (gamepad1.a) {
            pos -= 0.01;
        } else if (gamepad1.b) {
            pos += 0.01;
        }

        pos = Math.min(Math.max(pos, 0), 0.5);
        servo.setPosition(Math.min(Math.max(pos, 0), 0.5));
        telemetry.addData("servo pos", servo.getPosition());
        telemetry.addData("desired pos", pos);
        telemetry.update();
        sleep(1);

        //leftServo

        /*pos = Math.min(Math.max(pos, 0.25), 2);
        servo.setPosition(Math.min(Math.max(pos, 0.25), 2));
        telemetry.addData("servo pos", servo.getPosition());
        telemetry.addData("desired pos", pos);
        telemetry.update();
        sleep(10);
        double pos = 0.5
        //rightServo*/
    }
}