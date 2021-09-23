package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.Map;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware_18458
{
    // controls the drive train
    public DcMotor  FL  = null;
    public DcMotor  BL  = null;
    public DcMotor  FR = null;
    public DcMotor  BR = null;

    //hwmap is the hardwaremap
    HardwareMap hwMap = null;
<<<<<<< HEAD
<<<<<<< HEAD
    private final ElapsedTime period = new ElapsedTime();
=======
    private ElapsedTime period = new ElapsedTime();
>>>>>>> 0d771e9 (Hardware for drivetrain)
=======
    private final ElapsedTime period = new ElapsedTime();
>>>>>>> 0d5b5ec (Created Drivetrain)

    public Hardware_18458(){

    }
    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // Be sure to comment out any parts that you do not have
        FL  = hwMap.get(DcMotor.class, "FL");
        FR = hwMap.get(DcMotor.class, "FR");
        BL = hwMap.get(DcMotor.class, "BL");
        BR  = hwMap.get(DcMotor.class, "BR");

        // Set motor directions
        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
    }
}