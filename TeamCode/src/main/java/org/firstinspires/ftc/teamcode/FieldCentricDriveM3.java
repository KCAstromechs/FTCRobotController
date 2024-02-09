package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FieldCentricDriveM3 (Java)")
public class FieldCentricDriveM3 extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private IMU imu;

    private DcMotor lift;

    private Servo leftGrabber;
    private Servo rightGrabber;

    private Servo planeLauncher;
    //private BNO055IMU imu;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;
        double yawAngle;
        double Speed_percentage;

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        imu = hardwareMap.get(IMU.class, "imu");

        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");

        lift = hardwareMap.get(DcMotor.class, "lift");

        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");


        // Put motor initialization code here

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Speed_percentage = 0.6;
        yawAngle = 0;
        // Initialize the IMU.
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        // Prompt user to press start button.
        telemetry.addData("IMU Example", "Press start to continue...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                telemetry.addData("Yaw", "Press Circle or B on Gamepad to reset.");
                // Check to see if reset yaw is requested.
                if (gamepad1.circle || gamepad1.b)   {
                    imu.resetYaw();
                }
                orientation = imu.getRobotYawPitchRollAngles();

// Booster Button!
                if (gamepad1.right_bumper || gamepad1.left_bumper) {
                    Speed_percentage = 1;
                } else {
                    Speed_percentage = 0.6;
                }

//                double robotInputY = gamepad1.left_stick_y;
//                double robotInputX = gamepad1.left_stick_x;

                yawAngle = orientation.getYaw(AngleUnit.RADIANS);

                // Uncomment this line when possible
                double theta = yawAngle;
                // PI / 2; = 90 degrees (in terms of radians)

                double _inputX = gamepad1.left_stick_x;
                double _inputY = gamepad1.left_stick_y;

                // Changing vectors of joystick input
                double robotInputY = ((_inputX * Math.sin(theta)) + (_inputY * Math.sin(theta + (PI / 2)))); // *1.4
                double robotInputX = (_inputX * Math.cos(theta)) + (_inputY * Math.cos(theta + (PI / 2)));


                // Robot-centric drive base code (with edits to robotInputY and robotInputX turn this into Field-centric drive)
                double backRightPower = (robotInputY + -robotInputX + gamepad1.right_stick_x) * Speed_percentage;
                double backLeftPower = (robotInputY + robotInputX + -gamepad1.right_stick_x) * Speed_percentage;
                double frontRightPower = (robotInputY + robotInputX + gamepad1.right_stick_x) * Speed_percentage;
                double frontLeftPower = (robotInputY + -robotInputX + -gamepad1.right_stick_x) * Speed_percentage;


                // A whole bunch of telemetry for testing... I didn't feel like deleting it yet
                //telemetry.addData("Power of backLeft", backLeftPower);
                //telemetry.addData("Power of backRight", backRightPower);
                //telemetry.addData("Power of frontLeft", frontLeftPower);
                //telemetry.addData("Power of frontRight", frontRightPower);

                //telemtry

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
                    planeLauncher.setPosition(.3);
                    telemetry.addData("planeLauncher", "launched");
                }
                // planeLauncher RESET
                if (gamepad2.a) {
                    planeLauncher.setPosition(0);
                }

                //current position (testing for autonomous)
                telemetry.addData("frontLeft Current Position", frontLeft.getCurrentPosition());

                telemetry.addData("stick_y", gamepad1.left_stick_y);

                telemetry.addData("robotInputY", robotInputY);
                telemetry.addData("leftYCos", gamepad1.left_stick_y * Math.cos(theta));
                telemetry.addData("leftXSin", gamepad1.left_stick_x * Math.sin(theta));


                telemetry.addData("stick_x", gamepad1.left_stick_x);

                telemetry.addData("robotInputX", robotInputX);
                telemetry.addData("leftYSin", gamepad1.left_stick_y * Math.sin(theta));
                telemetry.addData("LeftXCos", gamepad1.left_stick_x * Math.cos(theta));


                telemetry.addData("rightStickY", gamepad1.right_stick_y);
                telemetry.addData("rightStickX", gamepad1.right_stick_x);

                telemetry.addData("Yaw Angle", yawAngle);


                /* highestPower is the highest value out of all of the absolute values of
                backRightPower, backLeftPower, frontRightPower, and frontLeftPower. */
                double highestPower = Math.max(Math.max(Math.abs(backRightPower), Math.abs(backLeftPower)),
                        Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)));


                // Normalizing powers (the powers will never go above 1)
                if (highestPower > 1) {
                    backLeftPower = backLeftPower / highestPower;
                    backRightPower = backRightPower / highestPower;
                    frontLeftPower = frontLeftPower / highestPower;
                    frontRightPower = frontRightPower / highestPower;
                }

                // directional driving based on robot position
//                backRight.setPower((gamepad1.right_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger) * speedPercentage);
//                backLeft.setPower((gamepad1.left_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger) * speedPercentage);
//                frontRight.setPower((gamepad1.right_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger) * speedPercentage);
//                frontLeft.setPower((gamepad1.left_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger) * speedPercentage);

                backRight.setPower((backRightPower));
                backLeft.setPower((backLeftPower));
                frontRight.setPower((frontRightPower));
                frontLeft.setPower((frontLeftPower));
                telemetry.update();
                // Get the orientation and angular velocity.

                orientation = imu.getRobotYawPitchRollAngles();
                angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                telemetry.addData("Yaw Angle", JavaUtil.formatNumber(yawAngle, 2));;
                telemetry.update();
                // Lift
                lift.setPower(lift_power * lift_speed_limit);
            }
        }
    }
}