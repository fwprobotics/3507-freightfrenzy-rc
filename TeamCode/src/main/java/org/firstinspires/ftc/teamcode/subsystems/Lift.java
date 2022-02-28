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

    private boolean toggle = false;
    boolean pressed;
    boolean pressed2;
    private boolean horizExtend = false;
    public extensionOptions extensionOption = extensionOptions.RETRACT;


    public enum liftRunMode {
        AUTONOMOUS,
        TELEOP
    }

    public enum dropoffOptions {
        BOTTOM (0),
        MIDDLE (-560),
        TOP (-1290);

        public int position;
        dropoffOptions(int position) {this.position = position;}

        public int position() {return position;}
    }

    public enum extensionOptions {
        RETRACT (LiftConstants.leftRetract, LiftConstants.rightRetract),
        MIDDLE (LiftConstants.midLeftExt, LiftConstants.midRightExt),
        EXTEND (LiftConstants.leftExtend, LiftConstants.rightExtend);

        public double leftPos;
        public double rightPos;
        extensionOptions(double leftPos, double rightPos) {
            this.leftPos = leftPos;
            this.rightPos = rightPos;
        }

        public double leftPos() {return leftPos();}
        public double rightPos() {return rightPos();}
    }


    @Config
    public static class LiftConstants {
        public static double power_modifier = 0.5;
        public static double leftExtend = 0.43;
        public static double leftRetract = 1;

        public static double rightExtend = 0.63;
        public static double rightRetract = 0.2;

        public static double midLeftExt = 0.5;
        public static double midRightExt = 0.56;
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
                leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
            if (extensionOption == extensionOptions.EXTEND) {
                extensionOption = extensionOptions.RETRACT;
            } else {
                extensionOption = extensionOptions.EXTEND;
            }
            moveHorizLift(extensionOption);
        } else {
            pressed = false;
        }
    }

    public void midLiftToggle (boolean press){
        if (press) {
            if (!pressed2) {
                if (extensionOption == extensionOptions.MIDDLE) {
                    extensionOption = extensionOptions.RETRACT;
                } else {
                    extensionOption = extensionOptions.MIDDLE;
                }
                moveHorizLift(extensionOption);
            }
            pressed2 = true;
        } else {
            pressed2 = false;
        }
    }


    public void moveHorizLift(extensionOptions pos) {
        switch (pos) {
            case EXTEND:
                leftHorizLift.setPosition(LiftConstants.leftExtend);
                rightHorizLift.setPosition(LiftConstants.rightExtend);
                break;
            case MIDDLE:
                leftHorizLift.setPosition(LiftConstants.midLeftExt);
                rightHorizLift.setPosition(LiftConstants.midRightExt);
                break;
            case RETRACT:
            leftHorizLift.setPosition(LiftConstants.leftRetract);
            rightHorizLift.setPosition(LiftConstants.rightRetract);
            break;
        }
//        leftHorizLift.setPosition(pos.leftPos());
//        rightHorizLift.setPosition(pos.rightPos());
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
        rightLiftMotor.setTargetPosition(Pos.position());
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setPower(0.5);
        rightLiftMotor.setPower(0.5);
    }

    public void teleOpControl(double input, boolean up, boolean middle, boolean down, boolean horiz, boolean midextend) { // Rename inputs based on real buttons we choose
        horizontalLiftToggle(horiz);
        midLiftToggle(midextend);

        if (down) {
            setPosition(dropoffOptions.BOTTOM);
        }
        if (up) {
            setPosition(dropoffOptions.TOP);
        }
        if (middle) {
            setPosition(dropoffOptions.MIDDLE);
        }
        if (!down & !up & !middle  & !leftLiftMotor.isBusy() & !rightLiftMotor.isBusy()) {

            leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLiftMotor.setPower(input * LiftConstants.power_modifier);
            rightLiftMotor.setPower(input * LiftConstants.power_modifier); //Probably should/can get toned down
        }
        l.telemetry.addData("left encoder", leftLiftMotor.getCurrentPosition());
        l.telemetry.addData("right encoder", rightLiftMotor.getCurrentPosition());
    }
}
