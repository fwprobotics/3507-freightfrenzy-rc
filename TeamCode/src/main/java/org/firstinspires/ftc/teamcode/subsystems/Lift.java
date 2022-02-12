package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Lift {

    public DcMotor leftLiftMotor;
    public DcMotor rightLiftMotor;
    public Servo leftHorizLift;
    public Servo rightHorizLift;
    public LinearOpMode l;
    public Telemetry realTelemetry;

    //Don't use this
//    public double basePos;
//    public double bottomPos;
//    public double middlePos;
//    public double topPos;

    private boolean toggle = false;
    boolean pressed;
    private boolean horizExtend = false;


    public enum liftRunMode {
        AUTONOMOUS,
        TELEOP
    }

    public enum dropoffOptions {
        BASE (0),
        BOTTOM (200),
        MIDDLE (500),
        TOP (800);

        private int position;
        dropoffOptions(int position) {this.position = position;}

        private int position() {return position;}
    }


    @Config
    public static class LiftConstants {
        public static double power_modifier = 0.5;
        public static double leftExtend = 0.41;
        public static double leftRetract = 1;

        public static double rightExtend = 0.65;
        public static double rightRetract = 0.2;
    }

    public Lift(liftRunMode runmode, LinearOpMode Input, HardwareMap hardwareMap, Telemetry telemetry) {

        l = Input;
        realTelemetry = telemetry;

        leftLiftMotor = hardwareMap.dcMotor.get("leftLiftMotor");
        rightLiftMotor = hardwareMap.dcMotor.get("rightLiftMotor");
        leftHorizLift = hardwareMap.servo.get("leftHorizontalLiftServo");
        rightHorizLift = hardwareMap.servo.get("rightHorizontalLiftServo");

        // Different motor configurations depending on use case
        switch (runmode) {
           case AUTONOMOUS:

                leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftLiftMotor.setTargetPosition(0);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightLiftMotor.setTargetPosition(0);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;

            case TELEOP:
                leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse left side
        }
    }

    public void horizontalLiftToggle(boolean press) {
        if (press) {
            if (!pressed) {
                toggle = !toggle;
            }
            pressed = true;
            if (!toggle) {
                retractLift();
            } else {
                extendLift();
            }
        } else {
            pressed = false;
        }
    }

    public void extendLift() {
        leftHorizLift.setPosition(LiftConstants.leftExtend);
        rightHorizLift.setPosition(LiftConstants.rightExtend);
    }
    public void retractLift() {
        leftHorizLift.setPosition(LiftConstants.leftRetract);
        rightHorizLift.setPosition(LiftConstants.rightRetract);
    }


    public void jakeTempLiftControl(double input, boolean horiz) {
        leftLiftMotor.setPower(input * LiftConstants.power_modifier);
        rightLiftMotor.setPower(input * LiftConstants.power_modifier);
        horizontalLiftToggle(horiz);
        l.telemetry.addData("left encoder", leftLiftMotor.getCurrentPosition());
        l.telemetry.addData("right encoder", rightLiftMotor.getCurrentPosition());

    }

   public void setAutoPosition(dropoffOptions Pos){
        setPosition(Pos);
        while (leftLiftMotor.isBusy() || rightLiftMotor.isBusy()) {
            l.idle();
        }
    }

    public void setPosition(dropoffOptions Pos){
        leftLiftMotor.setTargetPosition(Pos.position());
        leftLiftMotor.setPower(0.5);
        rightLiftMotor.setTargetPosition(Pos.position());
        rightLiftMotor.setPower(0.5);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftLiftMotor.isBusy() || rightLiftMotor.isBusy()) {
            l.idle();
        }
    }

    public void teleOpControl(double input, boolean up, boolean mid, boolean down, boolean base) { // Rename inputs based on real buttons we choose
        if (down) {
            setPosition(dropoffOptions.BOTTOM);
        }
        if (up) {
            setPosition(dropoffOptions.TOP);
        }
        if (mid) {
            setPosition(dropoffOptions.MIDDLE);
        }
        if (base) {
            setPosition(dropoffOptions.BASE);
        }
        if (!down & !up & !mid & !base & !leftLiftMotor.isBusy() & !rightLiftMotor.isBusy()) {
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLiftMotor.setPower(input * LiftConstants.power_modifier); //Probably should/can get toned down
        }
    }


}
