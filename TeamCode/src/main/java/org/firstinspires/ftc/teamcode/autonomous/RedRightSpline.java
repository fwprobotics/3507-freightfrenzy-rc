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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.LoopyPipeline2;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Dumper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous
public class RedRightSpline extends LinearOpMode {

    OpenCvCamera webcam;
    LoopyPipeline2 pipeline;
    Drivetrain drivetrain;
    Dumper dumper;
    Intake intake;
    Lift lift;


    CupPosition cupPos;

    public enum CupPosition {
        LEFT, // A
        MIDDLE, // B
        RIGHT // C
    }
    @Override

    public void runOpMode() {
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drivetrain = new Drivetrain(this, hardwareMap, telemetry);
        dumper = new Dumper(this, hardwareMap, telemetry);
        intake = new Intake(this, hardwareMap, telemetry);
        lift = new Lift(Lift.liftRunMode.AUTONOMOUS,this, hardwareMap, telemetry);

        Pose2d startPose = new Pose2d(6, -66, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        initCV();
        dumper.autoMove(Dumper.dumpPositions.LITTLE);
        dumper.moveKicker(false);

        TrajectorySequence littleForward = drive.trajectorySequenceBuilder(startPose)
                .forward(2)
                .build();

        TrajectorySequence trajSecLow = drive.trajectorySequenceBuilder(littleForward.end())
                //Some tuning needed to account for different drop off levels
                .lineToSplineHeading(new Pose2d(7, -20, Math.toRadians(0)))
                .forward(3.5)
                // To the blue square if we must save time
                /* .strafeRight(30)
                .splineTo(new Vector2d(-66,36), Math.toRadians(270))
                */
                .build();
        TrajectorySequence trajSecMid = drive.trajectorySequenceBuilder(littleForward.end())
                //Some tuning needed to account for different drop off levels
                .lineToSplineHeading(new Pose2d(3, -20, Math.toRadians(0)))
                .forward(6.5)
                // To the blue square if we must save time
                /* .strafeRight(30)
                .splineTo(new Vector2d(-66,36), Math.toRadians(270))
                */
                .build();
        TrajectorySequence trajSecHigh = drive.trajectorySequenceBuilder(littleForward.end())
                //Some tuning needed to account for different drop off levels
                .lineToSplineHeading(new Pose2d(3, -20, Math.toRadians(0)))
                .forward(5)
                // To the blue square if we must save time
                /* .strafeRight(30)
                .splineTo(new Vector2d(-66,36), Math.toRadians(270))
                */
                .build();




        waitForStart();

        if(isStopRequested()) return;
        findCup();
        drive.followTrajectorySequence(littleForward);

        switch (cupPos) {
            case LEFT:
                drive.followTrajectorySequence(trajSecLow);
                break;
            case MIDDLE:
                lift.setAutoPosition(Lift.dropoffOptions.MIDDLE);
                drive.followTrajectorySequence(trajSecMid);
                break;
            case RIGHT:
                lift.setAutoPosition(Lift.dropoffOptions.TOP);
                drive.followTrajectorySequence(trajSecHigh);
                break;
        }


        lift.moveHorizLift(Lift.extensionOptions.MIDDLE);
        sleep(500);
        dumper.autoMove(Dumper.dumpPositions.DUMP);
        sleep(500);
        dumper.moveKicker(true);
        sleep(1000);
        dumper.moveKicker(false);
        sleep(1000);
        dumper.autoMove(Dumper.dumpPositions.LITTLE);
        lift.moveHorizLift(Lift.extensionOptions.RETRACT);
        sleep(500);
        lift.setAutoPosition(Lift.dropoffOptions.BOTTOM);
        dumper.autoMove(Dumper.dumpPositions.DOWN);

        drive.updatePoseEstimate();
        Pose2d NewPose = drive.getPoseEstimate();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder((NewPose))
                .lineToSplineHeading(new Pose2d(6,-72, 0))
                .forward(36)
                .strafeLeft(30)
                .forward(24)
                .build();

        drive.followTrajectorySequence(traj2);
//        intake.intakeStatus = Intake.intakeStatuses.ON;
//        intake.runIntake();
//
//        while (dumper.hasCube) {
//            drivetrain.JoystickMovement(0.5, 0, 0,false);
//            dumper.dumpModerator();
//        }
//        intake.intakeDirection = Intake.intakeDirections.REVERSE;
//        intake.runIntake();
//
//        drive.updatePoseEstimate();
//        Pose2d NewPose2 = drive.getPoseEstimate();
//        TrajectorySequence newTraj = drive.trajectorySequenceBuilder(NewPose2)
//                .back(60)
//                //Without odometry, there becomes error here as battery worsens
//                .splineTo(new Vector2d(-12, -42), Math.toRadians(90))
//                //Drop off
//                .build();
//
//        drive.followTrajectorySequence(newTraj);
//        intake.intakeStatus = Intake.intakeStatuses.OFF;
//        intake.runIntake();
//        lift.setAutoPosition(Lift.dropoffOptions.TOP);
//        lift.toMid();
//        dumper.autoMove(Dumper.dumpPositions.DUMP);
//        sleep(200);
//        dumper.moveKicker(true);
//        sleep(500);
//        dumper.moveKicker(false);
//        dumper.autoMove(Dumper.dumpPositions.LITTLE);
//        lift.retractLift();
//        lift.setAutoPosition(Lift.dropoffOptions.BOTTOM);
//        dumper.autoMove(Dumper.dumpPositions.DOWN);
//        drive.updatePoseEstimate();
//        Pose2d NewPose3 = drive.getPoseEstimate();
//        TrajectorySequence lasttraj = drive.trajectorySequenceBuilder(NewPose3)
//                .lineToSplineHeading(new Pose2d(-12,-72, 0))
//                .forward(60)
//                .build();
//        drive.followTrajectorySequence(lasttraj);


    }

    private void initCV() {
        // Sets variable for the camera id
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Gives a name to the webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Combines the above to create a webcam that we will use
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //Sets our pipeline to view images through as the one we want
        //(Boundary between regions 1 and 2, Boundary between 2 and 3, Far left, Far top, Far right, Far bottom, opmode, the side we're on)
        pipeline = new LoopyPipeline2(170, 255, 110, 125, 300, 190, this);
        webcam.setPipeline(pipeline);

        // Turns on the webcam
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
            //This is needed so it knows what to do if something goes wrong
            public void onError(int thing){
                telemetry.addData("error",thing );
            }

        });

    }

    private void findCup() {

//Takes some time so we can run everything through the pipeline
        sleep(3000);

        LoopyPipeline2.Position cupPosition = pipeline.position;

//Sets cupPos to a corresponding position basesd on pipeline analysis

        switch (cupPosition) {
            case LEFT:
                cupPos = CupPosition.LEFT;
                break;
            case MIDDLE:
                cupPos = CupPosition.MIDDLE;
                break;
            case RIGHT:
                cupPos = CupPosition.RIGHT;
                break;
        }

    }
}
