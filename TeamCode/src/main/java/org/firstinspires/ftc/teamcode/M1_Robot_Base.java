
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
/*
public class M1_Robot_Base {

    //Important Set-Up Stuff
    OpMode callingOpMode;

    public M1_Robot_Base(OpMode _callingOpMode) {
        callingOpMode = _callingOpMode;
    }

    //Declare OpMode members
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor encoderY = null;
    private BNO055IMU imu;


    public void init() throws InterruptedException{
        // Hardware Map Stuff
        backLeft = callingOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = callingOpMode.hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = callingOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = callingOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        encoderY = callingOpMode.hardwareMap.get(DcMotor.class, "encoderY");
        imu = callingOpMode.hardwareMap.get(BNO055IMU.class, "imu");

        //Reverse motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // FANCY BRAKE CODE
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // RESET ENCODERS
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // START THE ENCODERS
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // GYRO INITIALIZE
        float zAngle;
        float yAngle;
        float xAngle;
        xAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        BNO055IMU.Parameters IMUParams = new BNO055IMU.Parameters();IMUParams.mode = BNO055IMU.SensorMode.IMU;
        IMUParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(IMUParams);
        Thread.sleep(50);
*/
/*
    }
    }
    public void drive(double rightPower, double leftPower){
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
    }
    public void strafe(double rightPower, double leftPower){
        double trigger;
        trigger = (gamepad1.right_trigger - gamepad1.left_trigger) / 2;
        frontRight.setPower(rightPower - trigger);
        backRight.setPower(rightPower + trigger);
        frontRight.setPower(leftPower + trigger);
        backRight.setPower(leftPower - trigger);
    }
}

 */

