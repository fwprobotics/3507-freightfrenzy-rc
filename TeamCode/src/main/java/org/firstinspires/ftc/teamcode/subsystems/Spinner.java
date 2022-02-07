package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
Class for controlling intake on robot.
Single motor. Toggled with button press. Can switch directions.
By Jake, 1/27/20.
 */


public class Spinner {

    private DcMotor spinnerMotor;
    private LinearOpMode l;
    private Telemetry realTelemetry;

    public enum spinnerStatuses {
        ON,
        OFF
    }

    public enum spinnerDirections {
        FORWARD,
        REVERSE
    }

    private spinnerStatuses spinnerStatus = spinnerStatuses.OFF;
    public spinnerDirections spinnerDirection = spinnerDirections.FORWARD;
    private boolean toggle;
    private boolean toggle2;
    private int direction = -1;

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

    // Toggle on and off intake via inputButton
    public void toggleSpinner(boolean input) {
        if (input) {
            if (!toggle) {
                toggle = true;
                switch (spinnerStatus) {
                    case OFF:
                        spinnerStatus = Spinner.spinnerStatuses.ON;
                        break;
                    case ON:
                        spinnerStatus = Spinner.spinnerStatuses.OFF;
                        break;
                }
            }
        } else {
            toggle = false;
        }
    }

    // Sets power of intake depending on direction
    public void runSpinner() {
        switch (spinnerStatus) {
            case ON:
                spinnerMotor.setPower(SpinnerConstants.spinner_power * direction);
                break;
            case OFF:
                spinnerMotor.setPower(0);
                break;
        }
    }

    // While input button is held down intake is reversed
    public void directionControl(boolean input) {
        if (input) {
            if (!toggle2) {
                toggle2 = true;
                switch (spinnerDirection) {
                    case FORWARD:
                        spinnerDirection = Spinner.spinnerDirections.REVERSE;
                        direction = 1;
                        break;
                    case REVERSE:
                        spinnerDirection = Spinner.spinnerDirections.FORWARD;
                        direction = -1;
                        break;
                }
            }
        } else {
            toggle2 = false;
        }
    }

}
