// Specifically for the left blue starting position.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="VisionAutoLeftBlue (Java)")
public class VisionAutoLeftBlue extends LinearOpMode {

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


        VisionBase.COLOR SpikeMarkLeft = vision.findRGB(0, 52, 211, 275, true);
        VisionBase.COLOR SpikeMarkCenter = vision.findRGB(272, 344, 205, 275, true);
        VisionBase.COLOR SpikeMarkRight = vision.findRGB(572, 638, 230, 276, true);
        if (SpikeMarkLeft == VisionBase.COLOR.BLUE) {
            telemetry.addData("Final Answer", "Left BLUE");
            telemetry.update();
            sleep(4000);
            // Move forward about half a square
            MOVE_FORWARD(600);
            // Strafe left to line grabber up with team prop
            // Move forward (maybe) a tad bit
            // Place purple pixel on spike mark
            // Move backward a tad bit
            // Turn left 90 degrees
            // Move forward until touching backdrop
            // Place yellow pixel on board (should correspond with Blue Alliance Left April tag thingyaslkfa jsd;fj )
            // Back up a tad
            // Strafe left 3/4 of a square
            // Move forward to park
        }
        else if (SpikeMarkCenter == VisionBase.COLOR.BLUE) {

            telemetry.addData("Final Answer", "Center BLUE");
            telemetry.update();
            sleep(4000);
            // Move forward 1 square
            MOVE_FORWARD(1126);
            // Strafe left a bit
            // Move forward a TAD
            // Place purple pixel on spike mark
            // Move backward a TAd
            // Turn left 90 degrees
            // Move forward until touching backdrop
            // Place yellow pixel on backdrop (should correspond with Blue Alliance Center April tag thingasjdlf;kj)
            // Move backward a TAD
            // Strafe left 1 square
            // Move forward to park
        }
        else if (SpikeMarkRight == VisionBase.COLOR.BLUE) {
            telemetry.addData("Final Answer", "Right BLUE");
            telemetry.update();
            sleep(4000);
            // Move forward a TADDDD
            MOVE_FORWARD(100);
            // Turn right 90 degrees
            TURN_RIGHT(930);
            // Strafe left 1 square
            STRAFE_LEFT(1020);
            // Move forward a bit
            // Place purple pixel on spike mark
            // Move backward a bit
            // Turn 180 degrees (doesn't matter which way)
            // Move forward until touching backdrop
            // Place yellow pixel on backdrop (should correspond with Blue Alliance Right April tag things djfas;ldkfj )
            // Move backward a BIG tad
            // Strafe left 1.25 squares
            // Move forward to park
        }
        else {
            telemetry.addData("Final Answer", "BLUE NOT DETECTED... GOING CENTER");
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
            // Turn left 90 degrees
            // Move forward until touching backdrop
            // Place yellow pixel on backdrop (should correspond with Blue Alliance Center April tag thingasjdlf;kj)
            // Move backward a TAD
            // Strafe left 1 square
            // Move forward to park
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
}