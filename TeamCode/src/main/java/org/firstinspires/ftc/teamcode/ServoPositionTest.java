package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoPositionTest (Java)")

public class ServoPositionTest extends LinearOpMode {

    /*    Declare OpMode members     */
    private Servo leftGrabber; // can change leftGrabber
    private Servo rightGrabber; // can change rightGrabber

    @Override
    public void runOpMode() {

        // Get Servos
        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber"); // can change leftGrabber
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber"); // can change rightGrabber

        // wait for start
        waitForStart();

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                // Telemetry for the Servo positions
                telemetry.addData("leftGrabberPosition", leftGrabber.getPosition());
                telemetry.addData("leftGrabberPosition", rightGrabber.getPosition());

                telemetry.update();

            }
        }
    }
}
