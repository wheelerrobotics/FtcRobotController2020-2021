package org.firstinspires.ftc.teamcode.comp.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class Rrunr extends LinearOpMode {
    public static double f3 = 2;
    public static double f2 = 2;
    public static double f1 = 1;
    public static double r3 = -1;
    public static double r2 = -1;
    public static double r1 = 1;
    public static double m = 1;

    public static double t4 = 0;
    public static double t5 = 0;
    public static double t6 = 0;
    public static double t3 = 0;
    public static double t2 = 0;
    public static double t1 = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(f1, r1), t1)
                .splineTo(new Vector2d(f2, r2), t2)
                .build();

        Trajectory myT1 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(0, 24 * f1))
                .build();

        Trajectory myT2 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(24 * r1,0))
                .build();

        Trajectory myT3 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(0, 24 * f2))
                .build();

        Trajectory myT4 = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(24 * r2, 0))
                .build();




        waitForStart();

        if(isStopRequested()) return;
        if (m == 1) {
            drive.followTrajectory(myT1);
            drive.followTrajectory(myT2);
            drive.followTrajectory(myT3);
            drive.followTrajectory(myT4);
        }else {
            drive.followTrajectory(traj);
        }

    }
}
