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

        // When we say new Pose2d() What we're saying is that we want to assume the bot starts at
        // the center of the field facing straight (we'll discuss the coordinate system later). If
        // you want to set your starting position to something else, say x: 5, y: -4, at a 90 degree
        // turn, you would replace new Pose2d() with new Pose2d(5, -4, Math.toRadians(90)).
        Trajectory myTrajectroy = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                // Notice there is no ; this is bc the line doesn't actually end here but we make
                // it a new line to make it look nice.

                // be sure to check out the list of all of these builder functions
                .strafeRight(10)
                .forward(5)
                .build();
                // if you try to build this, you will get an error. This is because of a
                // Path Continuity Exception. The reason for this is that when you go from strafe
                // right to forward, the robot experiences a sudden turn  which is infinite
                // acceleration. Even if the robot had rocket boosters, it would still make
                // a small arc. To be able to follow this path, the bot needs to come to a full
                // stop before turning.
                // Read the learnroadrunner guide at
                // https://learnroadrunner.com/trajectories.html#path-continuity-exception
                // to learn some ways to fix this.
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory((myTrajectroy));
    }
}
