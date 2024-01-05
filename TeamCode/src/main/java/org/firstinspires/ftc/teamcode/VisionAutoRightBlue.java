// Specifically for the left blue starting position.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="VisionAutoRightBlue (Java)")
public class VisionAutoRightBlue extends LinearOpMode {

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

        // do this before match start
        vision.initVision();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // now let's run vision, full image is 640 x 480
        // values for left line up:
        VisionBase.COLOR SpikeMarkLeft = vision.findRGB(0, 52, 211, 260, true);
        VisionBase.COLOR SpikeMarkCenter = vision.findRGB(272, 344, 205, 265, true);
        VisionBase.COLOR SpikeMarkRight = vision.findRGB(572, 638, 230, 276, true);
        if (SpikeMarkLeft == VisionBase.COLOR.BLUE) {
            telemetry.addData("Final Answer", "Left BLUE");
            telemetry.update();
            sleep(4000);
            // Move forward a TADDDD
            MOVE_FORWARD(50);
            // Turn right 90 degrees
            TURN_RIGHT(935);
            // Strafe left 1 square
            STRAFE_LEFT(1001);
            // Move forward a bit (optional)
            MOVE_FORWARD(10);
            // Place purple pixel on spike mark
            // TODO complete above comment
            // Strafe left 1 square
            STRAFE_LEFT(1200);
            // Move backward (under the main gate) and park (4 squares)
            MOVE_BACKWARD(4000);
        }
        else if (SpikeMarkCenter == VisionBase.COLOR.BLUE) {
            telemetry.addData("Final Answer", "Center BLUE");
            telemetry.update();
            sleep(4000);
            // Move forward 1 square
            MOVE_FORWARD(1050);
            // Strafe right a bit
            STRAFE_RIGHT(150);
            // Turn right 180 degrees
            TURN_RIGHT(1870);
            // Place purple pixel on spike mark
            // TODO complete above comment
            // Move forward a TAd
            MOVE_FORWARD(20);
            // Strafe left 1 square
            STRAFE_LEFT(875);
            // Move backward 2 square
            MOVE_BACKWARD(1500);
            // Turn right 90 degrees
            TURN_RIGHT(930);
            // Move forward (under main gate) and park (5 squares)
            MOVE_FORWARD(4769);
        }
        else if (SpikeMarkRight == VisionBase.COLOR.BLUE) {
            telemetry.addData("Final Answer", "Right BLUE");
            telemetry.update();
            sleep(4000);
            // Move forward about half a square
            MOVE_FORWARD(600);
            // Strafe right to line grabber up with team prop
            STRAFE_RIGHT(600);
            // Turn left 180
            TURN_LEFT(1900);
            // Place purple pixel on spike mark
            // TODO complete above comment
            sleep(500);
            // Move forward a tad bit
            MOVE_FORWARD(50);
            // Strafe right about half a square
            STRAFE_RIGHT(600);
            // Move backward about 2 squares
            MOVE_BACKWARD(1500);
            // Turn right 90 degrees
            TURN_RIGHT(930);
            // Move forward (under main gate) to park (5 squares)
            MOVE_FORWARD(4769);
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
            // Move forward 1 square
            MOVE_FORWARD(1050);
            // Strafe right a bit
            STRAFE_RIGHT(150);
            // Turn right 180 degrees
            TURN_RIGHT(1870);
            // Place purple pixel on spike mark
            // TODO complete above comment
            // Move forward a TAd
            MOVE_FORWARD(20);
            // Strafe left 1 square
            STRAFE_LEFT(875);
            // Move backward 2 square
            MOVE_BACKWARD(1500);
            // Turn right 90 degrees
            TURN_RIGHT(930);
            // Move forward (under main gate) and park (5 squares)
            MOVE_FORWARD(4769);
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
}