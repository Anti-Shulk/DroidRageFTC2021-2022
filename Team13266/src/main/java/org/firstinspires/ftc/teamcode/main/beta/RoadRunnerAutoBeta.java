package org.firstinspires.ftc.teamcode.main.beta;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Drive13266;

@Autonomous(name="RoadRunnerAutoBeta", group="Beta")
public class RoadRunnerAutoBeta extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drive13266 drive = new Drive13266(hardwareMap);

        Trajectory myTrajectroy = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory((myTrajectroy));
    }
}
