package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Dumper {

    public Servo dumpServo;
    public Servo kicker;
    public ColorSensor color;
    public LinearOpMode l;
    public Telemetry realTelemetry;
    public dumpPositions dumpPosition;
    public Intake intake;

    boolean pressed;
    private boolean horizExtend = false;
    public boolean hasCube = false;


    public enum dumpPositions {
        DOWN (DumperConstants.base),
        LITTLE (DumperConstants.picked),
        DUMP (DumperConstants.drop);

        private double position;
        dumpPositions(double position) {this.position = position;}

        private double position() {return position;}
    }

    @Config
    public static class DumperConstants {
        public static double kickOff = 0.3;
        public static double base = 0.5;
        public static double picked = 0.6;
        public static double drop = 1;
    }


    public Dumper(LinearOpMode Input, HardwareMap hardwareMap, Intake intake, Telemetry telemetry) {

        l = Input;
        realTelemetry = telemetry;

        dumpServo = hardwareMap.servo.get("dumper");
        kicker = hardwareMap.servo.get("kicker");
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
            if (dumpPosition == dumpPositions.DUMP) {
                kicker.setPosition(DumperConstants.kickOff);
            } else {
                kicker.setPosition(0);
            }
            l.telemetry.addData("Have a block?", hasCube);
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
