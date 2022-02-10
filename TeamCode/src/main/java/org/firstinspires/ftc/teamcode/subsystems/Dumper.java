package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Dumper {

    public Servo dumpServo;
    public ColorSensor color;
    public LinearOpMode l;
    public Telemetry realTelemetry;
    public dumpPositions dumpPosition;
    public Intake intake;

    boolean pressed;
    private boolean horizExtend = false;
    public boolean hasCube = false;


    public enum dumpPositions {
        DOWN (0),
        LITTLE (0.1),
        DUMP (1);

        private double position;
        dumpPositions(double position) {this.position = position;}

        private double position() {return position;}
    }


    public Dumper(LinearOpMode Input, HardwareMap hardwareMap, Intake intake, Telemetry telemetry) {

        l = Input;
        realTelemetry = telemetry;

        dumpServo = hardwareMap.servo.get("dumper");
        color = hardwareMap.get(ColorSensor.class, "colorSensor");
        dumpPosition = dumpPositions.DOWN;

        intake = intake;
        // Different motor configurations depending on use case
    }

    public void dumpModerator() {
        while (l.opModeIsActive()) {
            if ((color.red() > 1000) && (color.green()>1000)) {
                hasCube = true;
                if (dumpPosition == dumpPositions.DOWN) {
                    dumpPosition = dumpPositions.LITTLE;
                }
            } else {
                hasCube = false;
                dumpPosition = dumpPositions.DOWN;
            }
            dumpServo.setPosition(dumpPosition.position());

            if (!(dumpPosition == dumpPositions.DOWN)) {
                intake.directionControl(true);
                intake.runIntake();
            }
        }
    }

    public void dumpToggle(boolean press) {
        if (press) {
            if (!pressed) {
                if (dumpPosition == dumpPositions.DUMP) {
                    dumpPosition = dumpPositions.DOWN;
                } else {
                    dumpPosition = dumpPositions.DUMP;
                }
            }
            pressed = true;
        } else {
            pressed = false;
        }

    }


}
