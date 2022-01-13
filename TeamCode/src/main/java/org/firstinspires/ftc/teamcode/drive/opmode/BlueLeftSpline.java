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
public class BlueLeftSpline extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // It takes a really long time before this runs for some reason
        Pose2d startPose = new Pose2d(6, 66, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        Trajectory myTrajectory1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-12, 40), Math.toRadians(270))
                .build();
        Trajectory myTrajectory2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                //Improve this to make it less choppy
                .splineTo(new Vector2d(-8, 55), Math.toRadians(45))
                .splineTo(new Vector2d(0, 68), Math.toRadians(10))
                //.splineTo(new Vector2d(10, 76), Math.toRadians(0))
                .splineTo(new Vector2d(48, 68), Math.toRadians(0))
                .build();
        Trajectory myTrajectory3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(44, 54), Math.toRadians(90))
                .splineTo(new Vector2d(40, 68.5), Math.toRadians(175))
                .splineTo(new Vector2d(12, 72), Math.toRadians(180))
                .splineTo(new Vector2d(-12, 40), Math.toRadians(270))
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory1);
        sleep(1000);
        drive.followTrajectory(myTrajectory2);
        sleep(1000);
        drive.followTrajectory(myTrajectory3);
        sleep(1000);
        drive.followTrajectory(myTrajectory2);

    }
}
