package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class MercuryBaseExample {

    final double K_TURN = 0.02;
    final double STRAFE_CONSTANT = 0.02;
    final double CRONCHER_OPEN = 0.5;
    final double CRONCHER_CLOSED = 0;
    final double RIGHTHOOK_OPEN = .25;
    final double RIGHTHOOK_CLOSED = 1;
    final double LEFTHOOK_OPEN = .5;
    final double LEFTHOOK_CLOSED = 0;
    boolean speedy = false;

    //Important Set-Up Stuff
    OpMode callingOpMode;
    public MercuryBaseExample(OpMode _callingOpMode) {
        callingOpMode = _callingOpMode;
    }

    //Declare OpMode members
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor encoderY = null;
    private Servo leftServo = null;
    private Servo croncherServo = null;
    private Servo rightServo = null;
    private BNO055IMU imu;

    public void init() throws InterruptedException {
        // Hardware Map Stuff
        backLeft =  callingOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = callingOpMode.hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = callingOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = callingOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        encoderY = callingOpMode.hardwareMap.get(DcMotor.class, "encoderY");
        rightServo = callingOpMode.hardwareMap.get(Servo.class, "rightServo");
        leftServo = callingOpMode.hardwareMap.get(Servo.class, "leftServo");
        croncherServo = callingOpMode.hardwareMap.get(Servo.class, "croncherServo");
        imu = callingOpMode.hardwareMap.get(BNO055IMU.class,"imu");

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

        // the croncher should always start down but just in case...
        croncherCLOSE();

    }
    public void angleEncoderTele(){
        float yAngle;
        yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        callingOpMode.telemetry.addData( "y angle:", yAngle );
        callingOpMode.telemetry.update();
    }

    //Foundation Movers
    public void foundationOPEN() {
        rightServo.setPosition(RIGHTHOOK_OPEN);
        leftServo.setPosition(LEFTHOOK_OPEN);
    }

    public void foundationCLOSE() {
        rightServo.setPosition(RIGHTHOOK_CLOSED);
        leftServo.setPosition(LEFTHOOK_CLOSED);
    }

    //Croncher
    public void croncherOPEN(){
        croncherServo.setPosition(CRONCHER_OPEN);
    }
    public void croncherCLOSE(){
        croncherServo.setPosition(CRONCHER_CLOSED);
    }

    //Robot Broken
    public void yikesDrive() throws InterruptedException {
        frontRight.setPower(-.4);
        frontLeft.setPower(-.4);
        backRight.setPower(-.4);
        backLeft.setPower(-.4);
        Thread.sleep(750);

    }

    //Oh Yeah Its Strafing Time Babyyyy
    public void rightStrafe (int encoderClicks, double desiredAngle, double power) throws InterruptedException {
        float yAngle;
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

        frontRight.setPower(-power);
        backLeft.setPower(-power);
        frontLeft.setPower(power);
        backRight.setPower(power);

        while ( Math.abs(encoderClicks) > Math.abs(frontRight.getCurrentPosition() )){
            yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
            double driveCorrect = (yAngle - desiredAngle) * STRAFE_CONSTANT;
            frontRight.setPower(-power - driveCorrect);
            frontLeft.setPower(power + driveCorrect);
            backRight.setPower(power - driveCorrect);
            backLeft.setPower(-power + driveCorrect);

            callingOpMode.telemetry.addData( "frontRight:", frontRight.getCurrentPosition());
            callingOpMode.telemetry.update();
        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
    }

    public void leftStrafe(int encoderClicks, double desiredAngle, double power) throws InterruptedException {
        float yAngle;
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

        frontRight.setPower(power);
        backLeft.setPower(power);
        frontLeft.setPower(-power);
        backRight.setPower(-power);

        while ( Math.abs(encoderClicks) > Math.abs(frontRight.getCurrentPosition() )){
            yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
            double driveCorrect = (yAngle - desiredAngle) * STRAFE_CONSTANT;
            frontRight.setPower(power - driveCorrect);
            frontLeft.setPower(-power + driveCorrect);
            backRight.setPower(-power - driveCorrect);
            backLeft.setPower(power + driveCorrect);
        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
    }

    public void driveStraightInches(double inches, double desiredAngle, double power) throws InterruptedException {
        driveStraight((int)(inches*147.5), desiredAngle,power);
    }

    // All of the drive straight code which allows us to stay on course
    public void driveStraight(int encoderClicks, double desiredAngle, double power) throws InterruptedException {
        driveStraight (encoderClicks, desiredAngle,  power, false);
    }

    public void driveStraight(int encoderClicks, double desiredAngle, double power, boolean useCheat) throws InterruptedException {

        if(useCheat){
            desiredAngle = normalizeAngle(desiredAngle+180);
        }
        float yAngle = 0.0f;
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

        frontRight.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        backLeft.setPower(power);

        while ( Math.abs(encoderClicks) > Math.abs(encoderY.getCurrentPosition() )){
            yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
            if(useCheat) {
                yAngle = (float) normalizeAngle(yAngle + 180);
            }
            // TODO: Issue with angles crossing the circle discontinutiy
            //       e.g. yAngle = 171 and desiredAngle = -179
            double driveCorrect = (yAngle - desiredAngle) * K_TURN;
            frontRight.setPower(power - driveCorrect);
            backRight.setPower(power - driveCorrect);
            frontLeft.setPower(power + driveCorrect);
            backLeft.setPower(power + driveCorrect);

            callingOpMode.telemetry.addData( "encoderY:", encoderY.getCurrentPosition());
            callingOpMode.telemetry.update();
        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
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
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            frontRight.setPower(power);
            backRight.setPower(power);
            frontLeft.setPower(-power);
            backLeft.setPower(-power);

        } else {
            frontRight.setPower(-power);
            backRight.setPower(-power);
            frontLeft.setPower(power);
            backLeft.setPower(power);
        }


        // while the
        while (angleDifference(desiredAngle, yAngle) > 5) {
            yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;

           if(useCheat) {
               yAngle = (float)normalizeAngle(yAngle + 180);
           }
            Thread.sleep(10);
            callingOpMode.telemetry.addData( "y angle:", yAngle );
            callingOpMode.telemetry.update();


        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
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


}
