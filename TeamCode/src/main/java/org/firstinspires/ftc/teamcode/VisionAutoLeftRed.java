// Specifically for the left blue starting position.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@Autonomous(name="VisionAutoLeftRed (Java)")
public class VisionAutoLeftRed extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;

    double speed;

    @Override
    public void runOpMode() {
        VisionBase vision = new VisionBase(hardwareMap, telemetry);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        // Initialize motor settings (and speed)
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        speed = 0.3;

        CLOSE_GRABBER();

        // do this before match start
        vision.initVision();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Move the lift out of the way of the camera
        lift_move(true, -400);

        // now let's run vision, full image is 640 x 480
        // values for left line up:
        VisionBase.COLOR SpikeMarkLeft = vision.findRGB(20, 90, 75, 105, true);
        VisionBase.COLOR SpikeMarkCenter = vision.findRGB(284, 342, 64, 98, true);
        VisionBase.COLOR SpikeMarkRight = vision.findRGB(515, 581, 64, 95, true);
        if (SpikeMarkLeft == VisionBase.COLOR.RED) {
            telemetry.addData("Final Answer", "Left RED");
            telemetry.update();
            sleep(4000);
            // Move forward about half a square
            MOVE_FORWARD(600);
            // Strafe left to line grabber up with team prop
            // Move forward (maybe) a tad bit
            // Place purple pixel on spike mark
            // Move backward a tad bit
            // Strafe left about half a square
            // Move forward about 2 squares
            // Turn right 90 degrees
            // Move forward (under main gate) to park (5 squares)
        }
        else if (SpikeMarkCenter == VisionBase.COLOR.RED) {
            telemetry.addData("Final Answer", "Center RED");
            telemetry.update();
            sleep(4000);
            // Move forward 1 square
            MOVE_FORWARD(1126);
            // Strafe left a bit
            // Move forward a TAD
            // Place purple pixel on spike mark
            // Move backward a TAd
            // Strafe left 1 square
            // Move forward 2 square
            // Turn right 90 degrees
            // Move forward (under main gate) and park (5 squares)
        }
        else if (SpikeMarkRight == VisionBase.COLOR.RED) {
            telemetry.addData("Final Answer", "Right RED");
            telemetry.update();
            sleep(4000);
            // Move forward a TADDDD
            MOVE_FORWARD(100);
            // Turn right 90 degrees
            TURN_RIGHT(930);
            // Strafe left 1 square
            STRAFE_LEFT(1100);
            // Move forward a bit
            // Place purple pixel on spike mark
            // Move backward a bit
            // Strafe left 1 square
            // Move forward (under the main gate) and park (4 squares)
        }
        else {
            telemetry.addData("Final Answer", "RED NOT DETECTED");
            telemetry.addData("What we actually saw on the left", SpikeMarkLeft);
            telemetry.addData("What we actually saw in the center", SpikeMarkCenter);
            telemetry.addData("What we actually saw on the right", SpikeMarkRight);
            telemetry.update();
            sleep(4000);
            // Move forward 1 square
            MOVE_FORWARD(1126);
            // Strafe left a bit
            // Move forward a TAD
            // Place purple pixel on spike mark
            // Move backward a TAd
            // Strafe left 1 square
            // Move forward 2 square
            // Turn right 90 degrees
            // Move forward (under main gate) and park (5 squares)
        }

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
     * Turn right certain # of encoder clicks (in terms of front_left)
     * @param distanceEncoders
     */
    private void TURN_RIGHT(int distanceEncoders) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);
        while (Math.abs(frontLeft.getCurrentPosition()) < distanceEncoders) {
            telemetry.addData("frontLeft.getCurrentPosition()", frontLeft.getCurrentPosition());
            telemetry.update();
        }
        STOP_ROBOT();
    }


    /**
     * Turn left certain # of encoder clicks (in terms of front_left)
     * @param distanceEncoders
     */
    private void TURN_LEFT(int distanceEncoders) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);
        while (Math.abs(frontLeft.getCurrentPosition()) < distanceEncoders) {
            telemetry.addData("frontLeft.getCurrentPosition()", frontLeft.getCurrentPosition());
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