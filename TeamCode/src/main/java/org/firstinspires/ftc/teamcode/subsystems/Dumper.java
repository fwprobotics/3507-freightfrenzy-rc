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
    public dumpPositions lastDump;
    //public Intake intake;

    boolean pressed;
    public boolean toggle2;
    boolean pressed2;
    private boolean horizExtend = false;
    public boolean hasCube = false;


    public enum dumpPositions {
        DOWN (DumperConstants.base),
        LITTLE (DumperConstants.mid),
        DUMP (DumperConstants.drop);

        private double position;
        dumpPositions(double position) {this.position = position;}

        private double position() {return position;}
    }

    @Config
    public static class DumperConstants {
        public static double kickOff = 0.3;
        public static double base = 1;
        public static double mid = 0.6;
        public static double drop = 0.2;
    }


    public Dumper(LinearOpMode Input, HardwareMap hardwareMap, Intake intake, Telemetry telemetry) {

        l = Input;
        realTelemetry = telemetry;

        dumpServo = hardwareMap.servo.get("dumper");
        kicker = hardwareMap.servo.get("kicker");
        color = hardwareMap.get(ColorSensor.class, "colorSensor");
        dumpPosition = dumpPositions.DOWN;

        // intake = intake;
        // Different motor configurations depending on use case
    }

    public void dumpModerator() {
            if ((color.red() > 1000) && (color.green()>1000)) {
                hasCube = true;
//                if (dumpPosition == dumpPositions.DOWN) {
//                    dumpPosition = dumpPositions.LITTLE;
//                }
            } else {
                hasCube = false;
            }
            dumpServo.setPosition(dumpPosition.position());

            if (!(dumpPosition == dumpPositions.DOWN)) {
//                intake.directionControl(true);
//                intake.runIntake();
            }
            if (dumpPosition == dumpPositions.DUMP) {
                kicker.setPosition(DumperConstants.kickOff);
            } else {
                kicker.setPosition(0);
            }
            l.telemetry.addData("Have a block?", hasCube);
    }

    public void dumpToggle(boolean press) {
        if (press) {
            if (!pressed) {
                if (dumpPosition == dumpPositions.LITTLE) {
                    if (lastDump == dumpPositions.DUMP) {
                        dumpPosition = dumpPositions.DOWN;
                        lastDump = dumpPositions.DOWN;
                    } else {
                        dumpPosition = dumpPositions.DUMP;
                        lastDump = dumpPositions.DUMP;
                    }
                } else {
                    dumpPosition = dumpPositions.LITTLE;
                }
            }
            pressed = true;
            dumpServo.setPosition(dumpPosition.position());
        } else {
            pressed = false;
        }
    }

        public void midToggle(boolean press) {
            if (press) {
                if (!pressed2) {
                    toggle2 = !toggle2;
                }
                pressed2 = true;
                if (!toggle2) {
                    dumpPosition = dumpPositions.LITTLE;
                    kicker.setPosition(DumperConstants.kickOff);
                }
                dumpServo.setPosition(dumpPosition.position());
            } else {
                pressed2 = false;
            }



    }


}
