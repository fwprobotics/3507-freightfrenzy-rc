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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Dumper;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.LoopyPipeline2;
import org.firstinspires.ftc.teamcode.subsystems.Spinner;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class BlueRightSpline extends LinearOpMode {


    OpenCvCamera webcam;
    LoopyPipeline2 pipeline;
    Drivetrain drivetrain;
    Dumper dumper;
    Intake intake;
    Lift lift;
    Spinner spinner;

    CupPosition cupPos;

    public enum CupPosition {
        LEFT, // A
        MIDDLE, // B
        RIGHT // C
    }
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new Drivetrain(this, hardwareMap, telemetry);
        dumper = new Dumper(this, hardwareMap, telemetry);
        intake = new Intake(this, hardwareMap, telemetry);
        lift = new Lift(Lift.liftRunMode.AUTONOMOUS,this, hardwareMap, telemetry);
        spinner = new Spinner(this, hardwareMap, telemetry);


        Pose2d startPose = new Pose2d(-42, 66, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        initCV();
        dumper.autoMove(Dumper.dumpPositions.LITTLE);
        dumper.moveKicker(false);

        TrajectorySequence trajSec1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-57, 58, Math.toRadians(210)))
                .build();
                //Spin Duck
//                .waitSeconds(4)
//                //Some tuning needed to account for different drop off levels
//                .lineToSplineHeading(new Pose2d(-29, 24, Math.toRadians(180)))
//                //Drop off
//                .waitSeconds(2)
//
//                //To the blue square
//                .lineToSplineHeading(new Pose2d(-66, 36, Math.toRadians(0)))
//
//
//                .build();


        waitForStart();

        if(isStopRequested()) return;
        drive.followTrajectorySequence(trajSec1);
        spinner.spinnerStatus = Spinner.spinnerStatuses.ON;
//        spinner.spinnerDirection = Spinner.spinnerDirections.REVERSE;
        spinner.runSpinner();
        sleep(4000);
        drive.updatePoseEstimate();
        Pose2d NewPose = drive.getPoseEstimate();
        TrajectorySequence trajsec2 = drive.trajectorySequenceBuilder(NewPose)
                .splineTo(new Vector2d(-30, 24), Math.toRadians(180))
                .build();
        switch (cupPos) {
            case MIDDLE:
                lift.setAutoPosition(Lift.dropoffOptions.MIDDLE);
                break;
            case RIGHT:
                lift.setAutoPosition(Lift.dropoffOptions.TOP);
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
        Pose2d NewPose2 = drive.getPoseEstimate();
        TrajectorySequence trajsec3 = drive.trajectorySequenceBuilder(NewPose2)
                .lineToConstantHeading(new Vector2d(-66, 36))
                .build();





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
        pipeline = new LoopyPipeline2(70, 130, 50, 110, 225, 180, this);
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
