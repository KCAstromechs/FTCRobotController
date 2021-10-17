
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class M1_Robot_Base extends AstromechsRobotBase implements TankDriveable, Strafeable {

    //Important Set-Up Stuff
    DcMotor _frontLeft;
    DcMotor _backLeft;
    DcMotor _frontRight;
    DcMotor _backRight;
    DcMotor _encoderY;
    double _leftPower;
    double _rightPower;
    double _strafePower;
    final double K_TURN = 0.02;
    BNO055IMU imu;

    //thing that happens when new is used (constructor)
    public M1_Robot_Base(HardwareMap hardwareMap) {

        //underscore means it's a private variable

        _frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        _frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        _backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        _backRight = hardwareMap.get(DcMotor.class,"backRight");

        _frontRight.setDirection(DcMotor.Direction.REVERSE);
        _backRight.setDirection(DcMotor.Direction.REVERSE);

        _backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // RESET ENCODERS
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // START THE ENCODERS
        _frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _encoderY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    }

    /**
     * sets the power of the left side of the robot
     * @param power
     */
    @Override
    public void setLeftPower(double power){
        _leftPower = power;
    }

    /**
     * sets the power of the right side of the robot
     * @param power
     */
    @Override
    public void setRightPower(double power){
        _rightPower = power;
    }

    /**
     * sets power of the left and right sides of the robot
     *
     * @param leftPower
     * @param rightPower
     */
    @Override
    public void setSidePowers(double leftPower, double rightPower){
        setLeftPower(leftPower);
        setRightPower(rightPower);
    }

    @Override
    public void strafe(double strafePower){
        _strafePower=strafePower;


    }


    public void driveStraightInches(double inches, double desiredAngle, double power) throws InterruptedException{
        driveStraight((int)(inches*147.5), desiredAngle,power);
    }

    // All of the drive straight code which allows us to stay on course
    public void driveStraight(int encoderClicks, double desiredAngle, double power) throws InterruptedException{
        driveStraight (encoderClicks, desiredAngle,  power, false);
    }

    public void driveStraight(int encoderClicks, double desiredAngle, double power, boolean useCheat) throws InterruptedException {

        if(useCheat){
            desiredAngle = normalizeAngle(desiredAngle+180);
        }
        float yAngle = 0.0f;
        //_encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //_encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

        _frontRight.setPower(power);
        _backRight.setPower(power);
        _frontLeft.setPower(power);
        _backLeft.setPower(power);

        while ( Math.abs(encoderClicks) > Math.abs(_encoderY.getCurrentPosition() )){
            yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
            if(useCheat) {
                yAngle = (float) normalizeAngle(yAngle + 180);
            }
            // TODO: Issue with angles crossing the circle discontinutiy
            //       e.g. yAngle = 171 and desiredAngle = -179
            double driveCorrect = (yAngle - desiredAngle) * K_TURN;
            _frontRight.setPower(power - driveCorrect);
            _backRight.setPower(power - driveCorrect);
            _frontLeft.setPower(power + driveCorrect);
            _backLeft.setPower(power + driveCorrect);

        }

        _frontRight.setPower(0);
        _backRight.setPower(0);
        _frontLeft.setPower(0);
        _backLeft.setPower(0);
    }

    public void turnToAngle( double desiredAngle, double power ) throws InterruptedException {
        turnToAngle( desiredAngle, power, false );
    }

    // Turn to angle code
    public void turnToAngle( double desiredAngle, double power, boolean useCheat ) throws InterruptedException {

        float zAngle;
        float yAngle;
        float xAngle;

        //follow one motor's encoder count
       _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

        // intialize all of the angles
        yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;  // 0-90-180-
        // use the cheat?
        if(useCheat){
            yAngle = (float)normalizeAngle(yAngle+180);
            desiredAngle = normalizeAngle(desiredAngle+180);
        }
        // if the angle we want is more than the  angle we are at, spin until you get there
        if (desiredAngle>yAngle) {
            _frontRight.setPower(power);
            _backRight.setPower(power);
            _frontLeft.setPower(-power);
            _backLeft.setPower(-power);

        } else {
            _frontRight.setPower(-power);
            _backRight.setPower(-power);
            _frontLeft.setPower(power);
            _backLeft.setPower(power);
        }


        // while the
        while (angleDifference(desiredAngle, yAngle) > 5) {
            yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;

            if(useCheat) {
                yAngle = (float)normalizeAngle(yAngle + 180);
            }
            Thread.sleep(10);
        }

        _frontRight.setPower(0);
        _backRight.setPower(0);
        _frontLeft.setPower(0);
        _backLeft.setPower(0);
    }

    public double normalizeAngle(double ang1) {
        // Normal means 0-360
        double a = ang1;
        while (a > 360.0)
            a = a-360;
        while (a < 0.0)
            a = a+360;
        return a;
    }

    public double angleDifference(double ang1, double ang2) {
        double a1 = normalizeAngle(ang1);
        double a2 = normalizeAngle(ang2);

        if (Math.abs(a1 - a2) <= 180)
            return Math.abs(a1 - a2);

        if (Math.abs((a1+360) - a2) <= 180)
            return Math.abs((a1+360) - a2);

        return Math.abs(a1 - (a2+360));
    }


    @Override
    public void performUpdates() {
        //if strafePower (trigger) is 0 then it will act as a tank drive
        _frontRight.setPower(_rightPower-_strafePower);
        _backLeft.setPower(_leftPower-_strafePower);
        _frontLeft.setPower(_leftPower+_strafePower);
        _backRight.setPower(_rightPower+_strafePower);

    }
}