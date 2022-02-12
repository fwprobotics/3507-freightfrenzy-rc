package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
Class for controlling intake on robot.
Single motor. Toggled with button press. Can switch directions.
By Jake, 1/27/20.
 */


public class Intake {

    private DcMotor intakeMotor;
    private LinearOpMode l;
    private Telemetry realTelemetry;

    public enum intakeStatuses {
        //Intake at the power
        ON (1),
        //Do not intake
        OFF (0);

        private int status;
        intakeStatuses(int status) {this.status = status;}

        private int status() {return status;}
    }

    public enum intakeDirections {
        //Intake forward
        FORWARD (1),
        //Intake backward
        REVERSE (-1);

        private final int direction;
        intakeDirections(int direction) {this.direction = direction; }

        private int direction() {return direction;}
    }

    private intakeStatuses intakeStatus = intakeStatuses.OFF;
    public intakeDirections intakeDirection = intakeDirections.FORWARD;
    private boolean inputButtonPressed;

    @Config
    public static class IntakeConstants {
        public static double intake_power = 0.5;
    }

    public Intake(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry) {

        l = Input;
        realTelemetry = telemetry;
        realTelemetry.setAutoClear(true);

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    // TELEOP FUNCTIONS ------------------------

    // Toggle on and off intake via inputButton
    public void toggleIntake(boolean inputButton) {
        if (inputButton && !inputButtonPressed) {
            inputButtonPressed = true;
            switch (intakeStatus) {
                case OFF:
                    intakeStatus = intakeStatuses.ON;
                    break;
                case ON:
                    intakeStatus = intakeStatuses.OFF;
                    break;
            }
        }

        if (!inputButton) {
            inputButtonPressed = false;
        }
    }

    // Sets power of intake depending on direction
    public void runIntake() {
        //Gets value from whatever the direction and status enums are to determine direction and on/off
        intakeMotor.setPower(IntakeConstants.intake_power * intakeDirection.direction() * intakeStatus.status());
    }

    // While input button is held down intake is reversed
    // Direction is used to modifity output power (negative or positive 1)
    public void directionControl(boolean inputButton) {
        if (!inputButton) {
            intakeDirection = intakeDirections.FORWARD;
        } else {
            intakeDirection = intakeDirections.REVERSE;
        }

    }

}
