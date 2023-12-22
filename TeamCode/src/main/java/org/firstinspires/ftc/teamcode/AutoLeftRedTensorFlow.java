// AppData\Local\Android\Sdk\platform-tools\adb connect 192.168.43.1:5555
// Connects to robot through wi-fi
// AppData\Local\Android\Sdk\platform-tools\adb disconnect
// Disconnects from robot through wi-fi


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoLeftRedTensorFlow (Java)", preselectTeleOp = "TankDriveM2 (Java)")
public class AutoLeftRedTensorFlow extends LinearOpMode {

    boolean USE_WEBCAM;
    TfodProcessor myTfodProcessor;
    VisionPortal myVisionPortal;

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private Servo leftGrabber;
    private Servo rightGrabber;

    double speed;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");

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


        // This 2023-2024 OpMode illustrates the basics of TensorFlow Object Detection.
        USE_WEBCAM = true;
        // Initialize TFOD before waitForStart.
//        initTfod();
        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
//            MOVE_FORWARD(1300); move forward approx one square
            /**
             * Autonomous START
             */
            CLOSE_GRABBER();
            MOVE_FORWARD(2460);
            TURN_RIGHT(930); // 930 encoder clicks is about 90 degrees
            MOVE_FORWARD(4000);
            OPEN_GRABBER();
            MOVE_BACKWARD(100);
            /* New auto idea:
            Move forward 1 tile
            Check for pixel/team prop starting from center
            If pixel/team prop is in center, do this:
                Place purple pixel on center spike mark
                Strafe left 1 tile
                Forward 1 tile
                Turn right 90 degrees
                Forward ALL THE WAAYYYYYYY
                Let go of yellow pixel and scoot backward a teeny little bit
            Else, if pixel/team prop is on right spike mark, do this:
                Place purple pixel on right spike mark
                Turn back to original forward
                Forward 1 tile
                Turn right 90 degrees
                Forward ALL THE WAAYYYYYYY
                Let go of yellow pixel and scoot backward a teeny little bit
            Else, if pixel/team prop is on left spike mark, do this:
                Place purple pixel on left spike mark
                Turn back to original forward
                Forward 1 tile
                Turn right 90 degrees
                Forward ALL THE WAAYYYYYYY
                Let go of yellow pixel and scoot backward a teeny little bit
             */
            while (opModeIsActive()) {
//                // Put loop blocks here.
//                telemetryTfod();
//                // Push telemetry to the Driver Station.
//                telemetry.update();
//                if (gamepad1.dpad_down) {
//                    // Temporarily stop the streaming session.
//                    myVisionPortal.stopStreaming();
//                } else if (gamepad1.dpad_up) {
//                    // Resume the streaming session if previously stopped.
//                    myVisionPortal.resumeStreaming();
//                }
//                // Share the CPU.
//                sleep(20);
            }
        }
    }

//    /**
//     * Initialize TensorFlow Object Detection.
//     */
//    private void initTfod() {
//        TfodProcessor.Builder myTfodProcessorBuilder;
//        VisionPortal.Builder myVisionPortalBuilder;
//
//        // First, create a TfodProcessor.Builder.
//        myTfodProcessorBuilder = new TfodProcessor.Builder();
//        // Create a TfodProcessor by calling build.
//        myTfodProcessor = myTfodProcessorBuilder.build();
//        // Next, create a VisionPortal.Builder and set attributes related to the camera.
//        myVisionPortalBuilder = new VisionPortal.Builder();
//        if (USE_WEBCAM) {
//            // Use a webcam.
//            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        } else {
//            // Use the device's back camera.
//            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
//        }
//        // Add myTfodProcessor to the VisionPortal.Builder.
//        myVisionPortalBuilder.addProcessor(myTfodProcessor);
//        // Create a VisionPortal by calling build.
//        myVisionPortal = myVisionPortalBuilder.build();
//    }
//
//    /**
//     * Display info (using telemetry) for a detected object
//     */
//    private void telemetryTfod() {
//        List<Recognition> myTfodRecognitions;
//        Recognition myTfodRecognition;
//        float x;
//        float y;
//
//        // Get a list of recognitions from TFOD.
//        myTfodRecognitions = myTfodProcessor.getRecognitions();
//        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
//        // Iterate through list and call a function to display info for each recognized object.
//        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
//            myTfodRecognition = myTfodRecognition_item;
//            // Display info about the recognition.
//            telemetry.addLine("");
//            // Display label and confidence.
//            // Display the label and confidence for the recognition.
//            telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
//            // Display position.
//            x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
//            y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
//            // Display the position of the center of the detection boundary for the recognition
//            telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
//            // Display size
//            // Display the size of detection boundary for the recognition
//            telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
//        }
//    }

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
        rightGrabber.setPosition(0);
        leftGrabber.setPosition(.1);
        telemetry.addData("Grabber status", "open");
        telemetry.update();
    }


    /**
     * Close grabber
     */
    private void CLOSE_GRABBER() {
        rightGrabber.setPosition(.1);
        leftGrabber.setPosition(0);
        telemetry.addData("Grabber status", "closed");
        telemetry.update();
    }
}