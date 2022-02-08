package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp(name = "MecanumDrive", group = "TeleOp")
public class MecanumDrive extends LinearOpMode {

    Drivetrain drivetrain;
    Intake intake;
    Lift lift;

    @Override
    public void runOpMode() {

        //drivetrain = new Drivetrain(this, hardwareMap, telemetry);
        //intake = new Intake(this, hardwareMap, telemetry);
        lift = new Lift(Lift.liftRunMode.TELEOP, this, hardwareMap, telemetry);

        telemetry.addLine("Ready and WAITING :)");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        if (opModeIsActive()) {

            telemetry.clearAll();

            while (opModeIsActive()) {

               // drivetrain.JoystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper);

              //  intake.toggleIntake(gamepad1.b);
              //  intake.directionControl(gamepad1.a);
              //  intake.runIntake();

                lift.jakeTempLiftControl(gamepad2.right_stick_y, gamepad2.a);
                telemetry.update();

            }
        }
    }
}
