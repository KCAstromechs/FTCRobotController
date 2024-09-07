// Specifically for the left blue starting position.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="VisionAutoLeftRed (Java)", preselectTeleOp="TankDriveM3 (Java)")
public class VisionAutoLeftRed extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private Servo rightGrabber;
    private Servo leftGrabber;
    private DcMotor lift;

    private IMU imu_IMU;

    double speed;

    double yawAngle;

    @Override
    public void runOpMode() {


        imu_IMU = hardwareMap.get(IMU.class, "imu");

        VisionBase vision = new VisionBase(hardwareMap, telemetry);

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

        // Initialize the IMU.
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        yawAngle = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        CLOSE_GRABBER();

        // do this before match start
        vision.initVision();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Move the lift out of the way of the camera
        lift_move(true, -300);

        // now let's run vision, full image is 640 x 480
        // values for left line up:
        VisionBase.COLOR SpikeMarkLeft = vision.findRGB(20, 90, 75, 105, true);
        VisionBase.COLOR SpikeMarkCenter = vision.findRGB(284, 342, 64, 98, true);
//        VisionBase.COLOR SpikeMarkRight = vision.findRGB(515, 581, 64, 95, true);
        if (SpikeMarkLeft == VisionBase.COLOR.RED) {
            // TODO fine adjustments to be made
            telemetry.addData("Final Answer", "Left RED");
            telemetry.update();
            // Move forward a bit
            MOVE_FORWARD(500);
            // Strafe left half a square
            STRAFE_LEFT(600);
            // Move forward to place purple pixel
            MOVE_FORWARD(600);
            // Scoot back a bit
            MOVE_BACKWARD(400);
            // Strafe right half a square
            STRAFE_RIGHT(700);
            // Move forward so that the lift doesn't hit the wall
            MOVE_FORWARD(400);
            // Move the attachment dOWN To the GROUnd
            lift_move(true, -1350);
        }
        else if (SpikeMarkCenter == VisionBase.COLOR.RED) {
            // TODO fine adjustments to be made
            telemetry.addData("Final Answer", "Center BLUE");
            telemetry.update();

            // Move forward to place purple pixel on center spike mark
            MOVE_FORWARD(1570);
            // Scoot back a bit
            MOVE_BACKWARD(400);
            // Move the attachment dOWn to the GROUnd
            lift_move(true, -1350);
        }
        else {
            // TODO fine adjustments to be made
            telemetry.addData("Final Answer", "Right RED");
            telemetry.update();

            // Move forward to line up with right spike mark
            MOVE_FORWARD(1300);
            // Strafe left a little bit to avoid hitting the truss while turning
            STRAFE_LEFT(100);
            // Turn right 90 degrees to aim robot at right spike mark
            TURN_RIGHT(90);
            // Move forward a bit to get the purple pixel on the right spike mark
            MOVE_FORWARD(550);
            // Move backward a bit
            MOVE_BACKWARD(400);
            // Move the attachment dOWn to the GROUnd
            lift_move(true, -1350);
        }
//        else {
//            telemetry.addData("Final Answer", "RED NOT DETECTED... GOING CENTER");
//            telemetry.addData("What we actually saw on the left", SpikeMarkLeft);
//            telemetry.addLine();
//            telemetry.addData("What we actually saw in the center", SpikeMarkCenter);
//            telemetry.addLine();
//            telemetry.addData("What we actually saw on the right", SpikeMarkRight);
//            telemetry.update();
//
//            // Move forward to place purple pixel on center spike mark
//            MOVE_FORWARD(1570);
//            // Scoot back a bit
//            MOVE_BACKWARD(300);
//            // Move the attachment dOWn to the GROUnd
//            lift_move(true, -1350);
//        }

        telemetry.update();

        sleep(5000);

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
    }


    /**
     * Turn left certain # of degrees
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
    }


    /**
     * Open grabber
     */
    private void OPEN_GRABBER() {
        rightGrabber.setPosition(.05);
        leftGrabber.setPosition(.17);
        telemetry.addData("Grabber status", "open");
    }


    /**
     * Close grabber
     */
    private void CLOSE_GRABBER() {
        rightGrabber.setPosition(.15);
        leftGrabber.setPosition(.07);
        telemetry.addData("Grabber status", "closed");
    }


    /**
     * Moves lift to target position 'encoderPos'
     *
     * position -1350 is ground level
     *
     * position -800 is backdrop angle if backed all the way up the board
     *
     * @param encoderPos target position of lift in terms of encoders
     * @param lift_towards_back whether or not the lift it rotating towards the back of the robot from the starting position
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