package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest (Java)")

public class ServoTest extends LinearOpMode {

    private Servo planeLauncher;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double tgtPower = 0;

        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // put loop blocks here
                if (gamepad1.y) {
                    // move to 0 degrees
                    planeLauncher.setPosition(0);
                } else if (gamepad1.x || gamepad1.b) {
                    // move to 90 degrees
                    planeLauncher.setPosition(0.5);
                } else if (gamepad1.a) {
                    // move to 180 degrees
                    planeLauncher.setPosition(1);
                }
                telemetry.addData("Servo Position", planeLauncher.getPosition());
                telemetry.addData("Status", "Running");
                telemetry.update();
            }
        }
    }
}
