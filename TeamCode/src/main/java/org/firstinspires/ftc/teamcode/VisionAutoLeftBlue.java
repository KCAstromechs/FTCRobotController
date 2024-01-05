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


        VisionBase.COLOR SpikeMarkLeft = vision.findRGB(0, 52, 211, 260, true);
        VisionBase.COLOR SpikeMarkCenter = vision.findRGB(272, 344, 205, 265, true);
        VisionBase.COLOR SpikeMarkRight = vision.findRGB(572, 638, 230, 276, true);
        if (SpikeMarkLeft == VisionBase.COLOR.BLUE) {
            // TODO fine adjustments to be made
            telemetry.addData("Final Answer", "Left BLUE");
            telemetry.update();
            sleep(4000);
            // Move forward about half a square
            MOVE_FORWARD(600);
            // Strafe left to line grabber up with team prop
            STRAFE_LEFT(600);
            // Turn left 180 degrees
            TURN_LEFT(1865);
            // Place purple pixel on spike mark
            // TODO complete above comment
            // Turn left 90 degrees
            TURN_LEFT(930);
            // Strafe left a TAD
            STRAFE_LEFT(50);
            // Move backward until touching backdrop
            MOVE_BACKWARD(1000);
            // Place yellow pixel on board (should correspond with Blue Alliance Left April tag thingyaslkfa jsd;fj )
            // TODO complete above comment
            // Move forward a BIG tad
            MOVE_FORWARD(50);
            // Strafe right 3/4 of a square
            STRAFE_RIGHT(800);
            // Move backward to park
            MOVE_BACKWARD(550);
        }
        else if (SpikeMarkCenter == VisionBase.COLOR.BLUE) {
            // TODO fine adjustments to be made
            telemetry.addData("Final Answer", "Center BLUE");
            telemetry.update();
            sleep(4000);
            // Move forward 1 square
            MOVE_FORWARD(1126);
            // Turn left 180
            TURN_LEFT(1860);
            // Place purple pixel on spike mark
            // TODO complete comment above
            // Move forward a TAd
            MOVE_FORWARD(30);
            // Turn left 90 degrees
            TURN_LEFT(930);
            // Move backward until touching backdrop
            MOVE_BACKWARD(1400);
            // Place yellow pixel on backdrop (should correspond with Blue Alliance Center April tag thingasjdlf;kj)
            // TODO complete comment above
            // Move forward a TAD
            MOVE_FORWARD(50);
            // Strafe right 1 square
            STRAFE_RIGHT(1100);
            // Move backward to park
            MOVE_BACKWARD(700);
        }
        else if (SpikeMarkRight == VisionBase.COLOR.BLUE) {
            // TODO fine adjustments to be made
            telemetry.addData("Final Answer", "Right BLUE");
            telemetry.update();
            sleep(4000);
            // Move forward a TADDDD
            MOVE_FORWARD(100);
            // Turn left 90 degrees
            TURN_LEFT(930);
            // Strafe right 1 square
            STRAFE_RIGHT(1100);
            // Move forward a bit
            MOVE_FORWARD(50);
            // Place purple pixel on spike mark
            // TODO complete above comment
            // Move backward a bit (not necessary)
            MOVE_BACKWARD(20);
            // Turn 180 degrees (doesn't matter which way)
            TURN_RIGHT(1860);
            // Move backward until touching backdrop
            MOVE_BACKWARD(1400);
            // Place yellow pixel on backdrop (should correspond with Blue Alliance Right April tag things djfas;ldkfj )
            // TODO complete above comment
            // Move forward a BIG tad
            MOVE_FORWARD(50);
            // Strafe right 1.25 squares
            STRAFE_RIGHT(1200);
            // Move backward to park
            MOVE_BACKWARD(900);
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
            // Move forward 1 square
            MOVE_FORWARD(1126);
            // Turn left 180
            TURN_LEFT(1860);
            // Place purple pixel on spike mark
            // TODO complete comment above
            // Move forward a TAd
            MOVE_FORWARD(30);
            // Turn left 90 degrees
            TURN_LEFT(930);
            // Move backward until touching backdrop
            MOVE_BACKWARD(1400);
            // Place yellow pixel on backdrop (should correspond with Blue Alliance Center April tag thingasjdlf;kj)
            // TODO complete comment above
            // Move forward a TAD
            MOVE_FORWARD(50);
            // Strafe right 1 square
            STRAFE_RIGHT(1100);
            // Move backward to park
            MOVE_BACKWARD(700);
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