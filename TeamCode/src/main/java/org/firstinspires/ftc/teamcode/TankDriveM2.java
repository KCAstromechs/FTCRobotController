package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TankDriveM2 (Java)")

public class TankDriveM2 extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
//    private DcMotor hang; NOT USED
    private Servo planeLauncher;
    private Servo leftGrabber;
    private Servo rightGrabber;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double Speed_percentage;
//        double hang_speed; NOT USED

        // Drive motors
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        // Hang motor (attachment)
//        hang = hardwareMap.get(DcMotor.class, "hang"); (NO LONGER USED)

        // Servos (precision attachments)
        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");
        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");

        // lock planeLauncher temp
        planeLauncher.setPosition(.225);

        // Put initialization blocks here.
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); (NO LONGER USED)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        Speed_percentage = 0.6;
//        hang_speed = 0.1; NOT USED
        // Speed_percentage = 60% speed
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                double backRightPower = gamepad1.right_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger;
                double backLeftPower = gamepad1.left_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger;
                double frontRightPower = gamepad1.right_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger;
                double frontLeftPower = gamepad1.left_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger;

                telemetry.addData("Position of left grabber", leftGrabber.getPosition());
                telemetry.addData("Position of right grabber", rightGrabber.getPosition());
                telemetry.addData("Grabber status", "neutral");
                telemetry.addData("Power of backLeft", backLeft.getPower());
                telemetry.addData("Power of backRight", backRight.getPower());
                telemetry.addData("Power of frontLeft", frontLeft.getPower());
                telemetry.addData("Power of frontRight", frontRight.getPower());
                telemetry.addData("Position of planeLauncher", planeLauncher.getPosition());
                telemetry.addData("planeLauncher", "loaded");
                telemetry.addData("left trigger position", gamepad2.left_trigger);
                telemetry.addData("right trigger position", gamepad2.right_trigger);

                if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    Speed_percentage = 1;
                } else {
                    Speed_percentage = 0.6;
                }
                // Tank Drive (with trigger strafing)
                backRight.setPower((backRightPower) * Speed_percentage);
                backLeft.setPower((backLeftPower) * Speed_percentage);
                frontRight.setPower((frontRightPower) * Speed_percentage);
                frontLeft.setPower((frontLeftPower) * Speed_percentage);

                // Hang control (NO LONGER USED)
//                hang.setPower(gamepad2.left_trigger + -gamepad2.right_trigger);

                // planelaunch YES (launch)
                if (gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.ps) {
                    planeLauncher.setPosition(0);
                    telemetry.addData("planeLauncher", "launched");
                }
                // planeLauncher RESET
                if (gamepad2.a) {
                    planeLauncher.setPosition(.225);
                }

                // open grabber if either trigger is pressed enough
                if (gamepad2.left_trigger >= .5 || gamepad2.right_trigger >= .5) {
                    rightGrabber.setPosition(.1);
                    leftGrabber.setPosition(0);
                    telemetry.addData("Grabber status", "closed");
                } else {
                    rightGrabber.setPosition(0);
                    leftGrabber.setPosition(.1);
                    telemetry.addData("Grabber status", "open");
                }
                // Reset
//                if (gamepad2.x) {
//                    rightGrabber.setPosition(0);
//                    leftGrabber.setPosition(0);


                telemetry.update();

            }
        }
    }
}