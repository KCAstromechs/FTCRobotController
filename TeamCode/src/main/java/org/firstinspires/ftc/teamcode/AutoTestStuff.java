// Specifically for the left blue starting position.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="AutoTestStuff (Java)")
public class AutoTestStuff extends LinearOpMode {

    /* Declare OpMode members. */

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;

    private DcMotor lift;

    private Servo leftGrabber;
    private Servo rightGrabber;

    private IMU imu_IMU;

    double speed;
    double yawAngle;

    @Override
    public void runOpMode() {

        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;

        imu_IMU = hardwareMap.get(IMU.class, "imu");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        lift = hardwareMap.get(DcMotor.class, "lift");

        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");

        // Initialize motor settings (and speed)
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        speed = 0.3;
        yawAngle = 0;

        // Initialize the IMU.
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        TURN_LEFT(90);
        TURN_RIGHT(90);


        telemetry.addData("Yaw", "Press Circle or B on Gamepad to reset.");
        // Check to see if reset yaw is requested.
        if (gamepad1.circle) {
            imu_IMU.resetYaw();
        }
        // Get the orientation and angular velocity.
        orientation = imu_IMU.getRobotYawPitchRollAngles();
        angularVelocity = imu_IMU.getRobotAngularVelocity(AngleUnit.DEGREES);
        yawAngle = orientation.getYaw(AngleUnit.RADIANS);

        // Display yaw, pitch, and roll.
        telemetry.addData("Yaw (Z)", JavaUtil.formatNumber(orientation.getYaw(AngleUnit.DEGREES), 2));
        telemetry.addData("Pitch (X)", JavaUtil.formatNumber(orientation.getPitch(AngleUnit.DEGREES), 2));
        telemetry.addData("Roll (Y)", JavaUtil.formatNumber(orientation.getRoll(AngleUnit.DEGREES), 2));
        // Display angular velocity.
        telemetry.addData("Yaw (Z) velocity", JavaUtil.formatNumber(angularVelocity.zRotationRate, 2));
        telemetry.addData("Pitch (X) velocity", JavaUtil.formatNumber(angularVelocity.xRotationRate, 2));
        telemetry.addData("Roll (Y) velocity", JavaUtil.formatNumber(angularVelocity.yRotationRate, 2));
        telemetry.update();

    }
    /**
     * STOP the robot
     */
    private void STOP_ROBOT() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    /**
     * Move forward certain distance (in terms of front_left motor encoders
     * @param distanceEncoders
     */
    private void MOVE_FORWARD(int distanceEncoders) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
        while (Math.abs(frontLeft.getCurrentPosition()) < distanceEncoders) {
            telemetry.addData("frontLeft.getCurrentPosition()", frontLeft.getCurrentPosition());
            telemetry.update();
        }
        STOP_ROBOT();
    }

    /**
     * Move backward certain distance (in terms of front_left motor encoders)
     * @param distanceEncoders
     */
    private void MOVE_BACKWARD(int distanceEncoders) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);
        while (Math.abs(frontLeft.getCurrentPosition()) < distanceEncoders) {
            telemetry.addData("frontLeft.getCurrentPosition()", frontLeft.getCurrentPosition());
            telemetry.update();
        }
        STOP_ROBOT();
    }

    /**
     * Strafe right certain distance (in terms of front_left motor encoders)
     * @param distanceEncoders
     */
    private void STRAFE_RIGHT(int distanceEncoders) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);
        while (Math.abs(frontLeft.getCurrentPosition()) < distanceEncoders) {
            telemetry.addData("frontLeft.getCurrentPosition()", frontLeft.getCurrentPosition());
            telemetry.update();
        }
        STOP_ROBOT();
    }

    /**
     * Strafe left certain distance (in terms of front_left motor encoders)
     * @param distanceEncoders
     */
    private void STRAFE_LEFT(int distanceEncoders) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);
        while (Math.abs(frontLeft.getCurrentPosition()) < distanceEncoders) {
            telemetry.addData("frontLeft.getCurrentPosition()", frontLeft.getCurrentPosition());
            telemetry.update();
        }
        STOP_ROBOT();
    }

    /**
     * Turn right certain # of encoder clicks (in terms of front_left)
     * Turn right certain # of degrees
     * @param degrees
     */
    private void TURN_RIGHT(int degrees) {
        imu_IMU.resetYaw();
        yawAngle = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        sleep(200);
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);
        while (Math.abs(yawAngle) < degrees) {
            yawAngle = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("yawAngle", yawAngle);
            telemetry.update();
        }
        STOP_ROBOT();
//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        sleep(200);
//        frontLeft.setPower(speed);
//        frontRight.setPower(-speed);
//        backLeft.setPower(speed);
//        backRight.setPower(-speed);
//        while (Math.abs(frontLeft.getCurrentPosition()) < distanceEncoders) {
//            telemetry.addData("frontLeft.getCurrentPosition()", frontLeft.getCurrentPosition());
//            telemetry.update();
//        }
//        STOP_ROBOT();
    }


    /**
     * Turn left certain # of encoder clicks (in terms of front_left)
     * @param degrees
     */
    private void TURN_LEFT(int degrees) {
        imu_IMU.resetYaw();
        yawAngle = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        sleep(200);
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);
        while (Math.abs(yawAngle) < degrees) {
            yawAngle = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("yawAngle", yawAngle);
            telemetry.update();
        }
        STOP_ROBOT();

//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        sleep(200);
//        frontLeft.setPower(-speed);
//        frontRight.setPower(speed);
//        backLeft.setPower(-speed);
//        backRight.setPower(speed);
//        while (Math.abs(frontLeft.getCurrentPosition()) < distanceEncoders) {
//            telemetry.addData("frontLeft.getCurrentPosition()", frontLeft.getCurrentPosition());
//            telemetry.update();
//        }
//        STOP_ROBOT();
    }


    /**
     * Open grabber
     */
    private void OPEN_GRABBER() {
        rightGrabber.setPosition(.05);
        leftGrabber.setPosition(.17);
        telemetry.addData("Grabber status", "open");
        sleep(200);
    }


    /**
     * Close grabber
     */
    private void CLOSE_GRABBER() {
        rightGrabber.setPosition(.15);
        leftGrabber.setPosition(.07);
        telemetry.addData("Grabber status", "closed");
        sleep(200);
    }


    /**
     * Moves lift to target position 'encoderPos'
     * @param encoderPos target position of lift in terms of encoders
     * @param lift_towards_back whether or not the lift it rotating towards the back of the robot
     */
    private void lift_move(boolean lift_towards_back,int encoderPos) {
        sleep(200);
        if (lift_towards_back == true) {
            lift.setPower(-0.5);
            while (lift.getCurrentPosition() > encoderPos) {
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
            }
            lift.setPower(0);
        } else {
            lift.setPower(0.5);
            while (lift.getCurrentPosition() < encoderPos) {
                telemetry.addData("lift position", lift.getCurrentPosition());
                telemetry.update();
            }
            lift.setPower(0);
        }
    }
}
