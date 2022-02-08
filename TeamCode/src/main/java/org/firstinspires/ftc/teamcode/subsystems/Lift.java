package org.firstinspires.ftc.teamcode.subsystems;

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


    /*Correspond with encoder values for both motors.
    public enum dropoffOptions {
        BASE (),
        BOTTOM (),
        MIDDLE (),
        TOP ();

        private int position;
        dropoffOptions(int position) {this.position = position;}

        private int position() {return position;}
    }

     */

    public static class LiftConstants {
        public static double power_modifier = 0.7;
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
/*            case AUTONOMOUS:
                // TODO this needs serious work
                leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftLiftMotor.setTargetPosition(0);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightLiftMotor.setTargetPosition(0);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
                */
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
        } else {
            pressed = false;
        }

        if (!toggle) {
            retractLift();
        } else {
            extendLift();
        }
    }


    public void extendLift() {
        leftHorizLift.setPosition(0.3);
        rightHorizLift.setPosition(0.7);
    }
    public void retractLift() {
        leftHorizLift.setPosition(0.9);
        rightHorizLift.setPosition(0.1);
    }


    public void jakeTempLiftControl(double input, boolean horiz) {
        leftLiftMotor.setPower(input * LiftConstants.power_modifier);
        rightLiftMotor.setPower(input * LiftConstants.power_modifier);
        horizontalLiftToggle(horiz);
        l.telemetry.addData("left encoder", leftLiftMotor.getCurrentPosition());
        l.telemetry.addData("right encoder", rightLiftMotor.getCurrentPosition());
    }

//    public void setPosition(dropoffOptions position){
//        switch (position) {
//            case BASE:
//                leftLiftMotor.setTargetPosition(basePos); // Probably zero
//                break;
//            case BOTTOM:
//                leftLiftMotor.setTargetPosition(bottomPos);
//                break;
//            case MIDDLE:
//                leftLiftMotor.setTargetPosition(middlePos);
//                break;
//            case TOP:
//                leftLiftMotor.setTargetPosition(topPos);
//                break;
//        }
//        leftLiftMotor.setPower(1); //1 seems like a lot but we'll see
//        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }

//    public void teleOpControl(double input, boolean up, boolean mid, boolean down, boolean base){ // Rename inputs based on real buttons we choose
//       if down {
//            leftLiftMotor.setTargetPosition((int) bottomPos);
//            leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftLiftMotor.setPower(1);
//
//
//        }
//          if up {
//            armMotor.setTargetPosition(topPos);
//            leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftLiftMotor.setPower(1);
//        }
//        if mid{
//            armMotor.setTargetPosition(middlePos);
//            leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftLiftMotor.setPower(1);
//        }
//
//        if base {
//            armMotor.setTargetPosition(basePos);
//            leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftLiftMotor.setPower(1);
//        }
//        if (!down &! up &! mid &! base &! armMotor.isBusy()){
//        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftLiftMotor.setPower(input); //Probably should/can get toned down
//        }


}
