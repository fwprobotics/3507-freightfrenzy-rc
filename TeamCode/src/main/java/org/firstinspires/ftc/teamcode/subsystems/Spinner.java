package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
Spinner class. Very similar to intake.
 */


public class Spinner {

    private DcMotor spinnerMotor;
    private LinearOpMode l;
    private Telemetry realTelemetry;

    public enum spinnerStatuses {
        //Operate at full power
        ON(1),
        //Don't spin
        OFF(0);

        private int status;

        spinnerStatuses(int status) {
            this.status = status;
        }

        private int status() {
            return status;
        }
    }

    public enum spinnerDirections {
        FORWARD(1),
        REVERSE(-1);

        private int direction;

        spinnerDirections(int direction) {
            this.direction = direction;
        }

        private int direction() {
            return direction;
        }
    }

    private spinnerStatuses spinnerStatus = spinnerStatuses.OFF;
    public spinnerDirections spinnerDirection = spinnerDirections.FORWARD;
    private boolean toggle = false;
    private boolean pressed = false;

    public static class SpinnerConstants {
        public static double spinner_power = 1.0;
        public static double less_power = 0.5;
    }

    public Spinner(LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry) {

        l = Input;
        realTelemetry = telemetry;
        realTelemetry.setAutoClear(true);

        spinnerMotor = hardwareMap.dcMotor.get("spinnerMotor");

        spinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    // TELEOP FUNCTIONS ------------------------

    // Toggle on and off intake via buttons and y
    public void toggleSpinner(boolean x, boolean y) {

        //Changes direction based on which is pressed
        if (x &! y) {spinnerDirection=spinnerDirections.FORWARD;}
        if (y &! x) {spinnerDirection=spinnerDirections.REVERSE;}
        //if either are pressed, say a button is pressed and change the toggle it we haven't
        if (x || y) {
            if (!pressed) {
                toggle = !toggle;
            }
            pressed = true;
        } else {
            pressed = false;
        }

        //Turns the spinner on if we have toggled it on and off if we toggled it off
        if (toggle) {
            spinnerStatus=spinnerStatuses.ON;
        } else {
            spinnerStatus=spinnerStatuses.OFF;
        }
    }

    // Sets power of intake depending on direction
    public void runSpinner() {
        spinnerMotor.setPower(SpinnerConstants.spinner_power * spinnerDirection.direction() * spinnerStatus.status());
    }
}