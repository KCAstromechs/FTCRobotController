// AppData\Local\Android\Sdk\platform-tools\adb connect 192.168.43.1:5555
// Connects to robot through wi-fi
// AppData\Local\Android\Sdk\platform-tools\adb disconnect
// Disconnects from robot through wi-fi


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TankDriveM3 (Java)")

public class TankDriveM3 extends LinearOpMode {

    /* Declare OpMode members */
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor lift;

    private Servo leftGrabber;
    private Servo rightGrabber;
    private Servo planeLauncher;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        lift = hardwareMap.get(DcMotor.class, "lift");

        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");

        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");

        // Put initialization blocks here.
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        planeLauncher.setPosition(0);

        // Wait for start
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                // Naming powers and limits
                // Drive motors
                double backRightPower = gamepad1.right_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger;
                double backLeftPower = gamepad1.left_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger;
                double frontRightPower = gamepad1.right_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger;
                double frontLeftPower = gamepad1.left_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger;

                double Speed_percentage = .6; // Normal drive will be 60% of max power

                // Lift stuff
                double lift_speed_limit = .7; // Normal lift speed will be 50% of max power

                double lift_power = -gamepad2.right_stick_y;


                // Telemetry
                // Drive motor telemetry
                telemetry.addData("Power of backLeft", backLeft.getPower());
                telemetry.addData("Power of backRight", backRight.getPower());
                telemetry.addData("Power of frontLeft", frontLeft.getPower());
                telemetry.addData("Power of frontRight", frontRight.getPower());

                // Lift telemetry
                telemetry.addData("lift_power", lift.getPower());
                telemetry.addData("lift_position", lift.getCurrentPosition());
                /*
                For lift:
                Up is negative encoders
                Down is positive encoders
                 */

                // Grabber telemetry
                telemetry.addData("Position of left grabber", leftGrabber.getPosition());
                telemetry.addData("Position of right grabber", rightGrabber.getPosition());
                telemetry.addData("Grabber status", "neutral");

                // Plane Launcher telemetry
                telemetry.addData("Position of planeLauncher", planeLauncher.getPosition());
                telemetry.addData("planeLauncher", "loaded");

                // ------------------IF statements------------------

                // BOOSTER BUTTON!!!!!
                if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    Speed_percentage = 1;
                } else {
                    Speed_percentage = 0.6;
                }

                // GRABBER
                // close grabber if either bumpers/ trigger is pressed
                if (gamepad2.left_trigger >= .5 || gamepad2.right_trigger >= .5 || gamepad2.left_bumper || gamepad2.right_bumper) {
                    rightGrabber.setPosition(.15);
                    leftGrabber.setPosition(.07);
                    telemetry.addData("Grabber status", "closed");
                } else { // open grabber when bumpers/ triggers are not pressed
                    rightGrabber.setPosition(.05);
                    leftGrabber.setPosition(.17);
                    telemetry.addData("Grabber status", "open");
                }

                // Plane launcher
                // planelaunch YES (launch)
                if (gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.ps) {
                    planeLauncher.setPosition(.225);
                    telemetry.addData("planeLauncher", "launched");
                }
                // planeLauncher RESET
                if (gamepad2.a) {
                    planeLauncher.setPosition(0);
                }

                // Reset lift encoder


                // ------------------Set Powers------------------
                // Tank Drive (with trigger strafing)
                backRight.setPower((backRightPower) * Speed_percentage);
                backLeft.setPower((backLeftPower) * Speed_percentage);
                frontRight.setPower((frontRightPower) * Speed_percentage);
                frontLeft.setPower((frontLeftPower) * Speed_percentage);

                // Lift
                lift.setPower(lift_power * lift_speed_limit);


                telemetry.update();
            }
        }
    }
}