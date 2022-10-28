package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="FC", group="Iterative Opmode")
public class ApolloFCTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public ApolloBase rb;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    double fieldAngle;
    BNO055IMU imu;
    double K = 3;
    double desiredAngle;
    double direction = 0;
    boolean dpad = false;
    double dpadPower = 0.5;
    double errorK = 0.015;

    @Override
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rb = new ApolloBase(hardwareMap, telemetry, true);
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        BNO055IMU.Parameters IMUParams = new BNO055IMU.Parameters();
        IMUParams.mode = BNO055IMU.SensorMode.IMU;
        IMUParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu.initialize(IMUParams);
        while (!imu.isGyroCalibrated()) ;


        double waitTime = getRuntime() + 1.;
        //while(getRuntime()<waitTime);

        // original angle
        fieldAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Angle", fieldAngle);
        telemetry.update();


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */


    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        double zAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        if (dpad = true) {
            if (Math.abs(zAngle - desiredAngle) < errorK) {
                direction = 0;
                dpad = false;
            }
        }

        // if button toggle, switch speed
        if (gamepad1.left_bumper) {
            K = 5;
        } else if (gamepad1.right_bumper) {
            K = 3;
        }

        // testing idea
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            dpad = true;
            // what is our desired angle?
            if (gamepad1.dpad_up) {
                desiredAngle = 0;
            }
            else if (gamepad1.dpad_down) {
                desiredAngle = PI;
            }
            else if (gamepad1.dpad_left) {
                desiredAngle = -PI/2;
            }
            else if (gamepad1.dpad_right) {
                desiredAngle = PI/2;
            }
            else {
                desiredAngle = zAngle;
            }

            // are we at the angle?
            if (Math.abs(zAngle - desiredAngle) < errorK){
                direction = 0;
            }
            // which direction is fastest?
            else if (desiredAngle < zAngle) {
                direction = 1;
            } else if (desiredAngle > zAngle){
                direction = -1;
            }
            // discontinuity - are we going the shortest path?
            if (Math.abs(zAngle-desiredAngle) > PI) {
                direction *= -1;
            }
        }

        double turnPower = (gamepad1.left_stick_x);

        double inputX = gamepad1.right_stick_x;
        double inputY = -gamepad1.right_stick_y;

        double robotX = (inputX * Math.cos(zAngle)) + (inputY * Math.cos(zAngle + (PI / 2)));
        double robotY = (inputX * Math.sin(zAngle)) + (inputY * Math.sin(zAngle + (PI / 2)));

        double fLPower = ((-robotX + robotY) + turnPower - (dpadPower*direction)) / K;
        double fRPower = ((-robotX - robotY) - turnPower + (dpadPower*direction)) / K;
        double bRPower = ((-robotX + robotY) - turnPower + (dpadPower*direction)) / K;
        double bLPower = ((-robotX - robotY) + turnPower - (dpadPower*direction)) / K;

        double highestPowerF = Math.max(Math.abs(fLPower), Math.abs(fRPower));
        double highestPowerB = Math.max(Math.abs(bRPower), Math.abs(bLPower));
        double highestPower = Math.max(highestPowerB, highestPowerF);

        if (highestPower > 1) {
            fLPower /= highestPower;
            fRPower /= highestPower;
            bRPower /= highestPower;
            bLPower /= highestPower;
        }

        frontLeft.setPower(fLPower);
        frontRight.setPower(fRPower);
        backRight.setPower(bRPower);
        backLeft.setPower(bLPower);

        telemetry.addData("Desired Angle:", desiredAngle);
        telemetry.addData("zAngle:", zAngle);
        telemetry.addData("Direction:", direction);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop () {
    }

}

