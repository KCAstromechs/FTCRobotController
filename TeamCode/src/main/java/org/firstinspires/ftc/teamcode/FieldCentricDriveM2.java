package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FieldCentricDriveM2 (Java)")
public class FieldCentricDriveM2 extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private IMU imu;
    //private BNO055IMU imu;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;
        double yawAngle;
        double speedPercentage;

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        imu = hardwareMap.get(IMU.class, "imu");
        //imu = hardwareMap.get(BNO055IMU.class, "imu");



        // Put motor initialization code here
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        speedPercentage = 0.6;
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
                if (gamepad1.circle) {
                    imu.resetYaw();
                }
                orientation = imu.getRobotYawPitchRollAngles();
                angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                yawAngle = orientation.getYaw(AngleUnit.RADIANS);
// Booster Button!
                if (gamepad1.right_bumper || gamepad1.left_bumper) {
                    speedPercentage = 1;
                } else {
                    speedPercentage = 0.6;
                }

//                double robotInputY = gamepad1.left_stick_y;
//                double robotInputX = gamepad1.left_stick_x;

//                yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle;

                // Uncomment this line when possible
                double theta = yawAngle;

                // PI / 2; = 90 degrees (in terms of radians)
                // double theta = yawAngle

                double _inputX = gamepad1.left_stick_x;
                double _inputY = gamepad1.left_stick_y;

                // Commented out code for reasons (previous testing of field-centric drive)
//                double robotInputY = gamepad1.left_stick_y * Math.cos(theta) + gamepad1.left_stick_x * Math.sin(theta);
//                double robotInputX = gamepad1.left_stick_y * Math.sin(theta) + gamepad1.left_stick_x * Math.cos(theta);

                // Changing vectors of joystick input
                double robotInputY = ((_inputX * Math.sin(theta)) + (_inputY * Math.sin(theta + (PI / 2)))); // *1.4
                double robotInputX = (_inputX * Math.cos(theta)) + (_inputY * Math.cos(theta + (PI / 2)));


                // Robot-centric drive base code (with edits to robotInputY and robotInputX turn this into Field-centric drive)
                double backRightPower = (robotInputY + -robotInputX + gamepad1.right_stick_x) * speedPercentage;
                double backLeftPower = (robotInputY + robotInputX + -gamepad1.right_stick_x) * speedPercentage;
                double frontRightPower = (robotInputY + robotInputX + gamepad1.right_stick_x) * speedPercentage;
                double frontLeftPower = (robotInputY + -robotInputX + -gamepad1.right_stick_x) * speedPercentage;


                // A whole bunch of telemetry for testing... I didn't feel like deleting it yet
                //telemetry.addData("Power of backLeft", backLeftPower);
                //telemetry.addData("Power of backRight", backRightPower);
                //telemetry.addData("Power of frontLeft", frontLeftPower);
                //telemetry.addData("Power of frontRight", frontRightPower);

                //telemtry

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
                telemetry.addData("Yaw Angle", JavaUtil.formatNumber(yawAngle, 2));
                telemetry.addData("Yaw (Z)", JavaUtil.formatNumber(orientation.getYaw(AngleUnit.DEGREES), 2));
                telemetry.addData("Pitch (X)", JavaUtil.formatNumber(orientation.getPitch(AngleUnit.DEGREES), 2));
                telemetry.addData("Roll (Y)", JavaUtil.formatNumber(orientation.getRoll(AngleUnit.DEGREES), 2));
                // Display angular velocity.
                telemetry.addData("Yaw (Z) velocity", JavaUtil.formatNumber(angularVelocity.zRotationRate, 2));
                telemetry.addData("Pitch (X) velocity", JavaUtil.formatNumber(angularVelocity.xRotationRate, 2));
                telemetry.addData("Roll (Y) velocity", JavaUtil.formatNumber(angularVelocity.yRotationRate, 2));
                telemetry.update();
                // Display yaw, pitch, and roll.
            }
        }
    }
}