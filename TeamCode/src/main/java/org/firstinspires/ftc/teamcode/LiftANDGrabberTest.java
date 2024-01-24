package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "LiftANDGrabberTest (Java)")

public class LiftANDGrabberTest extends LinearOpMode {
    private DcMotor lift;
    private Servo leftGrabber;
    private Servo rightGrabber;

    @Override
    public void runOpMode() {

        lift = hardwareMap.get(DcMotor.class, "lift");
        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");

        // lift initialization
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // wait for start
        waitForStart();

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                double speed_limit = .3;

                double lift_power = gamepad1.right_stick_y;


                // If Statements
                // Lift safety
                if (lift.getCurrentPosition() > 0) {
                    lift_power = -Math.abs(lift_power);
                }

                // close grabber if either bumpers/ trigger is pressed
                if (gamepad2.left_trigger >= .5 || gamepad2.right_trigger >= .5 || gamepad2.left_bumper || gamepad2.right_bumper) {
                    rightGrabber.setPosition(.15);
                    leftGrabber.setPosition(.075);
                    telemetry.addData("Grabber status", "closed");
                    telemetry.addData("rightGrabberPos", rightGrabber.getPosition());
                    telemetry.addData("leftGrabberPos", leftGrabber.getPosition());
                } else { // open grabber
                    rightGrabber.setPosition(.05);
                    leftGrabber.setPosition(.175);
                    telemetry.addData("Grabber status", "open");
                    telemetry.addData("rightGrabberPos", rightGrabber.getPosition());
                    telemetry.addData("leftGrabberPos", leftGrabber.getPosition());
                }
                // Reset grabber
                if (gamepad2.x) {
                    rightGrabber.setPosition(0);
                    leftGrabber.setPosition(0);
                }


//                if (gamepad2.x) {
//                    rightGrabber.setPosition(0); // opens
//                    leftGrabber.setPosition(0); // closes
//                }
//
//                if (gamepad2.y) {
//                    rightGrabber.setPosition(0.1); // closes
//                    leftGrabber.setPosition(0.1); // opens
//                }
//
//                if (gamepad2.a) {
//                    rightGrabber.setPosition(0.1); // closes
//                    leftGrabber.setPosition(0); // closes
//                }
//
//                if (gamepad2.b) {
//                    rightGrabber.setPosition(0); // opens
//                    leftGrabber.setPosition(0.1); // opens
//                }


                lift.setPower(lift_power * speed_limit);

                telemetry.addData("lift_power", lift.getPower());
                telemetry.addData("lift_position", lift.getCurrentPosition());
                /*
                For lift:
                Up is negative encoders
                Down is positive encoders

                -900 encoders is going to be angle of board
                -1400 encoders is floor behind robot
                 */



                telemetry.update();
            }
        }

    }
}
