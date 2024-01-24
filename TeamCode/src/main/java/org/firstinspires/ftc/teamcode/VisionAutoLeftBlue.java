// Specifically for the left blue starting position.

package org.firstinspires.ftc.teamcode;

import android.mtp.MtpObjectInfo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.comp.Todo;

import org.checkerframework.common.value.qual.StringVal;


@Autonomous(name="VisionAutoLeftBlue (Java)", preselectTeleOp = "TankDriveM3 (Java)")
public class VisionAutoLeftBlue extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private Servo rightGrabber;
    private Servo leftGrabber;
    private DcMotor lift;

    double speed;

    @Override
    public void runOpMode() {
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

        CLOSE_GRABBER();

        // do this before match start
        vision.initVision();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Move the lift out of the way of the camera
        lift_move(true, -400);

        // now let's run vision, full image is 640 x 480
        VisionBase.COLOR SpikeMarkLeft = vision.findRGB(20, 90, 75, 105, true);
        VisionBase.COLOR SpikeMarkCenter = vision.findRGB(284, 342, 64, 98, true);
        VisionBase.COLOR SpikeMarkRight = vision.findRGB(515, 581, 64, 95, true);
        if (SpikeMarkLeft == VisionBase.COLOR.BLUE) {
            // TODO fine adjustments to be made
            telemetry.addData("Final Answer", "Left BLUE");
            telemetry.update();
            sleep(4000);
            // Strafe left half a square
            STRAFE_LEFT(600);
            // Move forward to place purple pixel
            MOVE_FORWARD(1100);
            // Scoot back a bit
            MOVE_BACKWARD(400);
            // Turn right 90 degrees to face back of robot at the backdrop
            TURN_RIGHT(1050);
            // Move backward towards the backdrop
            MOVE_BACKWARD(900);
            // Strafe left enough to line up with the backdrop
            STRAFE_LEFT(500);
            // Move backward into the backdrop
            MOVE_BACKWARD(400);
            // Move lift up to backdrop (IDK THE VALUE BUT WOOO)
            lift_move(true, -800);
            // Open grabber to drop yellow pixel
            OPEN_GRABBER();
            // Move lift away from board to avoid disturbance
            lift_move(false, -400);
            // Move forward a bit away from the board
            MOVE_FORWARD(200);
            // Strafe right a square to move away from the board
            STRAFE_RIGHT(1000);
            // Move backward to confirm park
            MOVE_BACKWARD(200);
        }
        else if (SpikeMarkCenter == VisionBase.COLOR.BLUE) {
            // TODO fine adjustments to be made
            telemetry.addData("Final Answer", "Center BLUE");
            telemetry.update();
            sleep(4000);
            // Move forward to place purple pixel on center spike mark
            MOVE_FORWARD(1510);
            // Scoot back a bit
            MOVE_BACKWARD(400);
            // Turn right 90 degrees to aim back of robot at backdrop
            TURN_RIGHT(1100);
            // Move backward into the backdrop
            MOVE_BACKWARD(1810);
            // Move lift up to backdrop (IDK THE VALUE BUT WOOO)
            lift_move(true, -800);
            // Open grabber to release yellow pixel
            OPEN_GRABBER();
            // Move lift away from board to avoid disturbance
            lift_move(false, -400);
            // Move forward a bit away from the board
            MOVE_FORWARD(200);
            // Strafe right a square to move away from the board
            STRAFE_RIGHT(1000);
            // Move backward to confirm park
            MOVE_BACKWARD(200);
        }
        else if (SpikeMarkRight == VisionBase.COLOR.BLUE) {
            // TODO fine adjustments to be made
            telemetry.addData("Final Answer", "Right BLUE");
            telemetry.update();
            sleep(4000);
            // Move forward to line up with right spike mark
            MOVE_FORWARD(1425);
            // Turn right 90 degrees to aim robot at right spike mark
            TURN_RIGHT(1000);
            // Move forward a bit to get the purple pixel on the right spike mark
            MOVE_FORWARD(500);
            // Move backward all the way to the back drop to place the yellow pixel
            MOVE_BACKWARD(2000);
            // Move lift up to backdrop (IDK THE VALUE BUT WOOO)
        }
        else {
            telemetry.addData("Final Answer", "BLUE NOT DETECTED... GOING CENTER");
            telemetry.addData("What we actually saw on the left", SpikeMarkLeft);
            telemetry.addLine();
            telemetry.addData("What we actually saw in the center", SpikeMarkCenter);
            telemetry.addLine();
            telemetry.addData("What we actually saw on the right", SpikeMarkRight);
            telemetry.update();
            sleep(4000);
            // TODO fine adjustments to be made
            telemetry.addData("Final Answer", "Center BLUE");
            telemetry.update();
            sleep(4000);
            // Move forward to place purple pixel on center spike mark
            MOVE_FORWARD(1500);
            // Scoot back a bit
            MOVE_BACKWARD(300);
            // Turn right 90 degrees to aim back of robot at backdrop
            TURN_RIGHT(1000);
            // Move backward into the backdrop
            MOVE_BACKWARD(1200);
            // Move lift up to backdrop (IDK THE VALUE BUT WOOO)
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
     * @param distanceEncoders degrees turned in terms of encoders (930 encoder clicks is about 90 degrees)
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
     * @param distanceEncoders degrees to turn in terms of encoders (930 encoder clicks is about 90 degrees)
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