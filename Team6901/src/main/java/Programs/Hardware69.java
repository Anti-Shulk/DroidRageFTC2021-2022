package Programs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware69
{



    /**
     * This is NOT an opmode.
     *
     * This class can be used to define all the specific hardware for a single robot.
     * In this case that robot is a Pushbot.
     * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
     *
     * This hardware class assumes the following device names have been configured on the robot:
     * Note:  All names are lower case and some have single spaces between words.
     *
     * Motor channel:  Left  drive motor:        "left_drive"
     * Motor channel:  Right drive motor:        "right_drive"
     * Motor channel:  Manipulator drive motor:  "left_arm"
     * Servo channel:  Servo to open left claw:  "left_hand"
     * Servo channel:  Servo to open right claw: "right_hand"
     */

        /* Public OpMode members. */
        public DcMotor  leftDrive   = null;
        public DcMotor  rightDrive  = null;
        public DcMotorEx  Arm     = null;
        public DcMotor intake =  null;
        public DcMotor backleftDrive = null;
        public DcMotor backrightDrive = null;
        public DcMotor Carousel =  null;

        public static final double MID_SERVO       =  0.5 ;
        public static final double ARM_UP_POWER    =  0.5 ;
        public static final double ARM_DOWN_POWER  = -0.5 ;
        public final double TICKS_PER_REV = 383.6;

        /* local OpMode members. */
        HardwareMap hwMap           =  null;
        private ElapsedTime period  = new ElapsedTime();

        /* Constructor */
        public Hardware69(){

        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize Motors
            leftDrive  = hwMap.get(DcMotor.class, "left_drive");
            rightDrive = hwMap.get(DcMotor.class, "right_drive");
            backleftDrive = hwMap.get(DcMotor.class, "BL_drive");
            backrightDrive = hwMap.get(DcMotor.class, "BR_drive");
            Arm    = hwMap.get(DcMotorEx.class, "arm");
            Carousel = hwMap.get(DcMotor.class, "Rotate");
           intake = hwMap.get(DcMotor.class, "intake");

            leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            backrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            Arm.setDirection(DcMotorSimple.Direction.REVERSE);
            Carousel.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setDirection(DcMotor.Direction.FORWARD);

            // set to brake mode
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Carousel.setDirection(DcMotorSimple.Direction.FORWARD);


            // Set all motors to zero power
            leftDrive.setPower(0);
            backleftDrive.setPower(0);
            rightDrive.setPower(0);
            backrightDrive.setPower(0);
            Arm.setPower(0);
            intake.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            Arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            Arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            Arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


            //Arm.setTargetPosition(0);
//            Arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);



           /* leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            */
            // Define and initialize ALL installed servos.

        }
    }

