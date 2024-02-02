package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "FieldCentricDriveM1 (Java)")

public class FieldCentricDriveM1 extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor attachment_1;

    private IMU imu_IMU;
    private BNO055IMU imu;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;

        double Speed_percentage;
        double yawAngle;

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        imu_IMU = hardwareMap.get(IMU.class, "imu");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        // Initialize the IMU.
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        // Prompt user to press start button.
        telemetry.addData("IMU Testing", "Press start to continue...");
        telemetry.update();

        // Put initialization blocks here.
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);



        Speed_percentage = 0.6;
        // Speed_percentage = 60% speed
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                // Booster Button!
                if (gamepad1.right_bumper || gamepad1.left_bumper) {
                    Speed_percentage = 1;
                } else {
                    Speed_percentage = 0.6;
                }

//                double robotInputY = gamepad1.left_stick_y;
//                double robotInputX = gamepad1.left_stick_x;

                yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle;

                // Uncomment this line when possible
                // double theta = yawAngle
                double theta = PI / 2;

                double _inputX = gamepad1.left_stick_x;
                double _inputY = gamepad1.left_stick_y;

                // Commented out code for reasons (previous testing of field-centric drive)
//                double robotInputY = gamepad1.left_stick_y * Math.cos(theta) + gamepad1.left_stick_x * Math.sin(theta);
//                double robotInputX = gamepad1.left_stick_y * Math.sin(theta) + gamepad1.left_stick_x * Math.cos(theta);

                // Changing vectors of joystick input
                double robotInputY = ((_inputX * Math.sin(theta)) + (_inputY * Math.sin(theta + (PI / 2))))*1.4;
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
//                backRight.setPower((gamepad1.right_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger) * Speed_percentage);
//                backLeft.setPower((gamepad1.left_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger) * Speed_percentage);
//                frontRight.setPower((gamepad1.right_stick_y + -gamepad1.left_trigger + gamepad1.right_trigger) * Speed_percentage);
//                frontLeft.setPower((gamepad1.left_stick_y + gamepad1.left_trigger + -gamepad1.right_trigger) * Speed_percentage);

                backRight.setPower((backRightPower));
                backLeft.setPower((backLeftPower));
                frontRight.setPower((frontRightPower));
                frontLeft.setPower((frontLeftPower));
                telemetry.update();
            }
        }
    }
}