package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class M2Base {

    final double K_TURN = 0.02;
    final double STRAFE_CONSTANT = 0.02;
    final double OPEN = .5;
    final double CLOSED = 1;

    //Important Set-Up Stuff
    OpMode callingOpMode;
    public M2Base(OpMode _callingOpMode) {
        callingOpMode = _callingOpMode;
    }

    //Declare OpMode members
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor wobbleGoal = null;
    private DcMotor intake = null;
    private DcMotor shooter = null;
    private DcMotor rotator = null;
    private Servo wobbleGrab = null;
    private BNO055IMU imu;


    public void init() throws InterruptedException {
        // Hardware Map Stuff
        backLeft =  callingOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = callingOpMode.hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = callingOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = callingOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        wobbleGoal = callingOpMode.hardwareMap.get(DcMotor.class, "wobbleGoal");
        shooter = callingOpMode.hardwareMap.get(DcMotor.class, "shooter");
        rotator = callingOpMode.hardwareMap.get(DcMotor.class, "rotator");
        intake = callingOpMode.hardwareMap.get(DcMotor.class,"intake");
        wobbleGrab = callingOpMode.hardwareMap.get(Servo.class, "wobbleGrab");
        imu = callingOpMode.hardwareMap.get(BNO055IMU.class,"imu");


        //Reverse motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // FANCY BRAKE CODE
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // RESET ENCODERS
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // START THE ENCODERS
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



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


    }
    public void shooter(double shooterPower){
        shooter.setPower(shooterPower);
    }

    public void rotator(double rotatorPower){
        rotator.setPower(rotatorPower);
    }

        //power always positive, if going backwards use negative encoder clicks
    public void wobbleGoalMech(double power, int clicks){
        wobbleGoal.setTargetPosition(wobbleGoal.getTargetPosition()+clicks);
        wobbleGoal.setPower(power);
    }

    public void wobbleServoPosition(String position){
        if(position =="CLOSED"){
            wobbleGrab.setPosition(CLOSED);
        }
        if (position == "OPEN"){
            wobbleGrab.setPosition(OPEN);
        }
    }


    public void angleEncoderServoTele(){
        float zAngle;
        zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        callingOpMode.telemetry.addData( "z angle:", zAngle );
        callingOpMode.telemetry.update();
    }

    //Oh Yeah Its Strafing Time Babyyyy
   /* public void rightStrafe(double inches, double desiredAngle, double power) throws InterruptedException{
        rightStrafeEncoder((int)(inches*147.5), desiredAngle, power);
    }

    public void rightStrafeEncoder (int encoderClicks, double desiredAngle, double power) throws InterruptedException{
        float zAngle;
        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

        frontRight.setPower(-power);
        backLeft.setPower(-power);
        frontLeft.setPower(power);
        backRight.setPower(power);

        while ( Math.abs(encoderClicks) > Math.abs(encoderX.getCurrentPosition() )){
            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            double driveCorrect = (zAngle - desiredAngle) * STRAFE_CONSTANT;
            frontRight.setPower(-power - driveCorrect);
            frontLeft.setPower(power + driveCorrect);
            backRight.setPower(power - driveCorrect);
            backLeft.setPower(-power + driveCorrect);

            callingOpMode.telemetry.addData( "encoderX:", encoderX.getCurrentPosition());
            callingOpMode.telemetry.update();
        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
    }

    public void leftStrafe(double inches, double desiredAngle, double power) throws InterruptedException{
        leftStrafeEncoder((int)(inches*147.5), desiredAngle, power);
    }

    public void leftStrafeEncoder(int encoderClicks, double desiredAngle, double power) throws InterruptedException{
        float zAngle;
        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

        frontRight.setPower(power);
        backLeft.setPower(power);
        frontLeft.setPower(-power);
        backRight.setPower(-power);

        while ( Math.abs(encoderClicks) > Math.abs(encoderX.getCurrentPosition() )){
            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            double driveCorrect = (zAngle - desiredAngle) * STRAFE_CONSTANT;
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
*/
    public void driveStraight(double inches, double desiredAngle, double power) throws InterruptedException {
        driveStraightEncoder((int)(inches*147.5), desiredAngle,power);
    }

    // All of the drive straight code which allows us to stay on course
    public void driveStraightEncoder(int encoderClicks, double desiredAngle, double power) throws InterruptedException {
       if(Math.abs(desiredAngle)>100){
           driveStraightEncoder(encoderClicks, desiredAngle, power, true);
       }
       else {
           driveStraightEncoder(encoderClicks, desiredAngle, power, false);
       }
    }

    public void driveStraightEncoder(int encoderClicks, double desiredAngle, double power, boolean useCheat) throws InterruptedException {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        Thread.sleep(250);

        if(useCheat){
            desiredAngle = normalizeAngle(desiredAngle);
        }
        float zAngle = 0.0f;
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

        frontRight.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        backLeft.setPower(power);

        while ( Math.abs(encoderClicks) > Math.abs(intake.getCurrentPosition() )){
            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            if(useCheat) {
                zAngle = (float) normalizeAngle(zAngle);
            }

            // TODO: Issue with angles crossing the circle discontinutiy
            //       e.g. zAngle = 171 and desiredAngle = -179
            double driveCorrect = (zAngle - desiredAngle) * K_TURN;
            frontRight.setPower(power - driveCorrect);
            backRight.setPower(power - driveCorrect);
            frontLeft.setPower(power + driveCorrect);
            backLeft.setPower(power + driveCorrect);

            callingOpMode.telemetry.addData( "encoderY:", intake.getCurrentPosition());
            callingOpMode.telemetry.addData("zAngle", zAngle);
            callingOpMode.telemetry.addData("desiredAngle", desiredAngle);
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
        zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;  // 0-90-180-
       // use the cheat?
        if(useCheat){
            zAngle = (float)normalizeAngle(zAngle+180);
            desiredAngle = normalizeAngle(desiredAngle+180);
        }
        // if the angle we want is more than the  angle we are at, spin until you get there
        if (desiredAngle>zAngle) {
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
        while (angleDifference(desiredAngle, zAngle) > 5) {
            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

           if(useCheat) {
               zAngle = (float)normalizeAngle(zAngle + 180);
           }
            Thread.sleep(10);
            callingOpMode.telemetry.addData( "y angle:", zAngle );
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
