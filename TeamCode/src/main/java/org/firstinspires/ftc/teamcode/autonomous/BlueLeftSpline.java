package org.firstinspires.ftc.teamcode.autonomous;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class BlueLeftSpline extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotorEx frontleft;
        DcMotorEx backleft;
        DcMotorEx backright;
        DcMotorEx frontright;

        frontleft = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        backleft = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        backright = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        frontright = hardwareMap.get(DcMotorEx.class, "frontRightDrive");

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        Pose2d startPose = new Pose2d(6, 66, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSec = drive.trajectorySequenceBuilder(startPose)
                //Find TSE position here
                .waitSeconds(3)
                //Some tuning needed to account for different drop off levels
                .lineToSplineHeading(new Pose2d(3, 24, Math.toRadians(0)))
                //Drop off
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(6,72, 0))
                .forward(28)
                //Move forward a semi-random extra amount
                /*
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontleft.setPower(.8);
                    frontright.setPower(.8);
                    backleft.setPower(.8);
                    backright.setPower(.8);

                })
                .waitSeconds(0.5+Math.random())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    frontleft.setPower(0);
                    frontright.setPower(0);
                    backleft.setPower(0);
                    backright.setPower(0);
                })
                //Intake

                .waitSeconds(3)
                //Should always bring us back to te same place
                .lineToConstantHeading(new Vector2d(-14,72))
                //Without odometry, there becomes error here as battery worsens
                .splineTo(new Vector2d(-16, 42), Math.toRadians(270))
                //Drop off
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(-12,72, 0))
                .forward(60)

                // To the blue square if we must save time
                /* .strafeRight(30)
                .splineTo(new Vector2d(-66,36), Math.toRadians(270))
                */
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajSec);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setPower(.4);
        frontright.setPower(.4);
        backleft.setPower(.4);
        backright.setPower(.4);


        sleep(500+Math.round(1000*Math.random()));
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
        drive.updatePoseEstimate();
        Pose2d NewPose = drive.getPoseEstimate();
        TrajectorySequence newTraj = drive.trajectorySequenceBuilder(NewPose)
                .waitSeconds(3)
                //Should always bring us back to te same place
                .lineToConstantHeading(new Vector2d(-14,72))
                //Without odometry, there becomes error here as battery worsens
                .splineTo(new Vector2d(-16, 42), Math.toRadians(270))
                //Drop off
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(-12,72, 0))
                .forward(60)
                .build();
        drive.followTrajectorySequence(newTraj);



    }
}