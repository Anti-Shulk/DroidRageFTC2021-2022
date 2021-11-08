package org.firstinspires.ftc.teamcode.main.beta;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config // adds all of the hardware into ftc dashboard
@TeleOp(name = "ArmTest")
public class ArmTest extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MotorEx leftFront;
    private DcMotorEx rightFront;

    @Override
    public void init() {
        leftFront = new MotorEx(hardwareMap, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront" );
    }
    @Override
    public void start() {
        runtime.reset();
    }
    @Override
    public void loop() {
        leftFront.set(1);
    }
}
