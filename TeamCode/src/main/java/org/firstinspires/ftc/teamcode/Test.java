package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import java.lang.Math;


@TeleOp(name="Test", group="Linear OpMode")
public class Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
//    private VoltageSensor voltageSensor = null;
//    vol= hardwareMap.voltageSensor.get("Battery_Voltage_Sensor");

    // IMU
    private IMU imu;

    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor frontLeft = null;

    private DcMotor intake = null;
    
    private DcMotor progression = null;
    private double progressionPercent = 1.0; // 1.0 = 100%

    private DcMotorEx outtakeLeft = null;
    private DcMotorEx outtakeRight = null;

    private double shooterPercent = .6; // 1.0 = 100%

    private double leftTicksPerRev;
    private double rightTicksPerRev;
    private double leftRPM;
    private double rightRPM;
    private boolean shooting = false;
    private boolean canShoot = false;

//    private boolean in = false;
//    private boolean progress = false;
//    private double voltage = voltageSensor.getVoltage();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // IMU stuff
        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;

        // CONSTANTS
        double yawAngle;
        double Speed_percentage;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");

        intake = hardwareMap.get(DcMotor.class, "intake");

        progression = hardwareMap.get(DcMotor.class, "progression");

        outtakeLeft = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "outtakeRight");

        leftTicksPerRev = outtakeLeft.getMotorType().getTicksPerRev();
        rightTicksPerRev = outtakeRight.getMotorType().getTicksPerRev();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);

        progression.setDirection(DcMotor.Direction.REVERSE);

        // To allow automatic braking, set 'zero power behavor' to brake for all motors
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        // Speed stuff
        outtakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        outtakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        outtakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Speed_percentage = 0.6;
        yawAngle = 0;
        // Initialize the IMU.
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        // Prompt user to press start button.
        telemetry.addData("IMU Example", "Press start to continue...");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Yaw/Rotation Angle", "Press Y on Gamepad to reset.");
            // Check to see if reset yaw is requested.
            if (gamepad1.y)   {
                imu.resetYaw();
            }
            orientation = imu.getRobotYawPitchRollAngles();

            yawAngle = orientation.getYaw(AngleUnit.RADIANS);

//            // intake/progression toggles
//
//            if (gamepad1.aWasPressed()) {
//                in = !in;
//            }

            // if driver1 triggers pressed, both on no matter what
            if ((shooting && canShoot) || gamepad2.x || Math.abs(gamepad1.right_trigger) > 0.5 || Math.abs(gamepad1.left_trigger) > 0.5) { // if toggled, intake in
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            // progression logic
            leftRPM = (outtakeLeft.getVelocity() / leftTicksPerRev) * 60;
            rightRPM = (outtakeRight.getVelocity() / rightTicksPerRev) * 60 * -1;
            shooting = gamepad2.right_trigger > 0.5;
            canShoot = (leftRPM > 95 && rightRPM > 95);
            if (gamepad2.x || gamepad2.a || (shooting && canShoot)) { // if toggled, progression continue
                progression.setPower(1 * progressionPercent);
            } else if (gamepad2.b) { // if b, progression retract from shooter
                progression.setPower(-1 * progressionPercent);
            } else {
                progression.setPower(0);
            }

            // shooter buttons
            if (gamepad2.dpadDownWasPressed()) {
                shooterPercent -= .05; // -5%
            } else if (gamepad2.dpadUpWasPressed()) {
                shooterPercent += .05; // +5%
            }

            // outtake
            outtakeLeft.setPower(gamepad2.right_trigger * shooterPercent);
            outtakeRight.setPower(-gamepad2.right_trigger * shooterPercent);

            // Ian's shooter thing
            /* if (leftRPM > 95 && rightRPM > 95) {
                progression.setPower(1);
                intake.setPower(1);
            } */

            // BOOSTER BUTTON!!!!!
            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                Speed_percentage = 1;
            } else {
                Speed_percentage = 0.6;
            }
            double theta = yawAngle;
            // PI / 2; = 90 degrees (in terms of radians)

            double _inputX = gamepad1.left_stick_x;
            double _inputY = gamepad1.left_stick_y;

            // Changing vectors of joystick input
            double robotInputY = ((_inputX * Math.sin(theta)) + (_inputY * Math.sin(theta + (PI / 2)))); // *1.4
            double robotInputX = (_inputX * Math.cos(theta)) + (_inputY * Math.cos(theta + (PI / 2)));

            // Robot-centric drive base code (with edits to robotInputY and robotInputX turn this into Field-centric drive)
            double rightBackPower = (robotInputY + -robotInputX + gamepad1.right_stick_x) * Speed_percentage;
            double leftBackPower = (robotInputY + robotInputX + -gamepad1.right_stick_x) * Speed_percentage;
            double rightFrontPower = (robotInputY + robotInputX + gamepad1.right_stick_x) * Speed_percentage;
            double leftFrontPower = (robotInputY + -robotInputX + -gamepad1.right_stick_x) * Speed_percentage;

            /* highestPower is the highest value out of all of the absolute values of
                rightBackPower, leftBackPower, rightFrontPower, and leftFrontPower. */
            double highestPower = Math.max(Math.max(Math.abs(rightBackPower), Math.abs(leftBackPower)),
                    Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)));

            // Normalizing powers (the powers will never go above 1)
            if (highestPower > 1) {
                leftBackPower = leftBackPower / highestPower;
                rightBackPower = rightBackPower / highestPower;
                leftFrontPower = leftFrontPower / highestPower;
                rightFrontPower = rightFrontPower / highestPower;
            }
            backRight.setPower((rightBackPower));
            backLeft.setPower((leftBackPower));
            frontRight.setPower((rightFrontPower));
            frontLeft.setPower((leftFrontPower));
            // Get the orientation and angular velocity.

            orientation = imu.getRobotYawPitchRollAngles();
            angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            telemetry.addData("Yaw Angle", JavaUtil.formatNumber(yawAngle, 2));

//            voltage = voltageSensor.getVoltage();
//            telemetry.addData("Battery Voltage", voltage);

            // Show the elapsed game time and wheel power.
//            telemetry.addData("Press A for intake toggle", in);
//            telemetry.addData("Press B for progression toggle", progress);
            telemetry.addData("Set Power of intake", intake.getPower());
            telemetry.addData("Set Power of progression", progression.getPower());
            telemetry.addData("Shooter Percentage", shooterPercent *100 + " %");
            telemetry.addData("RPM of shooterLeft", leftRPM); // (ticksPerSec/ticksPerRev) * 60
            telemetry.addData("RPM of shooterRight", rightRPM ); // (ticksPerSec/ticksPerRev) * 60sd
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
