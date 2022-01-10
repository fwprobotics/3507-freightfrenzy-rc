package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class SampleSpline extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(10, 72, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        Trajectory myTrajectory1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-9, 48), Math.toRadians(270))
                .build();
        Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory1.end())
                //Improve this to make it less choppy
                .splineTo(new Vector2d(-9, 60), Math.toRadians(0))
                .splineTo(new Vector2d(10, 76), Math.toRadians(0))
                .forward(48)
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory1);
        sleep(1000);
        drive.followTrajectory(myTrajectory2);

    }
}
