package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RedRightSpline extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(6, -66, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSec = drive.trajectorySequenceBuilder(startPose)
                //Find TSE position here
                .waitSeconds(3)
                //Some tuning needed to account for different drop off levels
                .lineToSplineHeading(new Pose2d(6, -24, Math.toRadians(0)))
                //Drop off
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(6,-72, 0))
                .forward(42)
                //Intake
                .waitSeconds(3)
                .back(60)
                //Without odometry, there becomes error here as battery worsens
                .splineTo(new Vector2d(-12, -42), Math.toRadians(90))
                //Drop off
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(-12,-72, 0))
                .forward(60)

                // To the blue square if we must save time
                /* .strafeRight(30)
                .splineTo(new Vector2d(-66,36), Math.toRadians(270))
                */
                .build();


        waitForStart();

        if(isStopRequested()) return;

//        drive.followTrajectory(myTrajectory1);
//        sleep(1000);
//
//        drive.followTrajectory(myTrajectory2);
//        sleep(1000);
//        drive.followTrajectory(myTrajectory3);
//        drive.followTrajectory(myTrajectory1);
//        sleep(1000);
//        drive.followTrajectory(myTrajectory2);

        drive.followTrajectorySequence(trajSec);



    }
}
