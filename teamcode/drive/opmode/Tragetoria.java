package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config

@Autonomous(group = "drive")

public class Tragetoria extends LinearOpMode {

    @Override

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(20)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(20)
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(20)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(20)
                .build();
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj3);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj4);
    }
}
