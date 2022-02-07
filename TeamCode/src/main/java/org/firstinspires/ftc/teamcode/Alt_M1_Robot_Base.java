
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Alt_M1_Robot_Base extends AstromechsRobotBase implements TankDriveable, Strafeable {

    public static final double duckPower = .50;
    final double autoDuckPower = .25;
    //Important Set-Up Stuff
    DcMotor _frontLeft;
    DcMotor _backLeft;
    DcMotor _frontRight;
    DcMotor _backRight;
    DcMotor _encoderY;
    DcMotor _carouselMover;

    DcMotor _lifter;

    double _leftPower;
    double _rightPower;
    double _strafePower;
    double _lifterPower;
    double _intakePower;
    Telemetry _telemetry;
    final double K_TURN = 0.02;
    BNO055IMU imu;
    final int lifterLevel1 =500;
    final int lifterLevel2 =973;
    final int lifterLevel3 = 1365;
    final double K_STRAFE = 0.001;
    public static final double DRIVE_STRAFE_ENCODER_TO_INCHES = 67;
    public static final double STRAFE_POWER_OFFSET = .3;

    //thing that happens when new is used (constructor)
    public Alt_M1_Robot_Base(HardwareMap hardwareMap, Telemetry telemetry) {

        //underscore means it's a private variable
        _telemetry = telemetry;

        _frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        _frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        _backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        _backRight = hardwareMap.get(DcMotor.class,"backRight");
        _encoderY = hardwareMap.get(DcMotor.class,"encoderY");
        _lifter= hardwareMap.get(DcMotor.class, "lifter");

        imu = hardwareMap.get(BNO055IMU.class,"imu");


        _frontRight.setDirection(DcMotor.Direction.REVERSE);
        _backRight.setDirection(DcMotor.Direction.REVERSE);
        _lifter.setDirection(DcMotor.Direction.REVERSE);

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
        _lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        // START THE ENCODERS
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _lifter.setTargetPosition(0);
        _lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        while(!imu.isGyroCalibrated());

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
        float zAngle = 0.0f;
        _encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

          _frontRight.setPower(power);
          _backRight.setPower(power);
          _frontLeft.setPower(power);
          _backLeft.setPower(power);

        while ( Math.abs(encoderClicks) > Math.abs(_encoderY.getCurrentPosition() )){
            information();
            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            if(useCheat) {
                zAngle = (float) normalizeAngle(zAngle + 180);
            }
            // TODO: Issue with angles crossing the circle discontinutiy
            //       e.g. yAngle = 171 and desiredAngle = -179
            double driveCorrect = (zAngle - desiredAngle) * K_TURN;
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

    public void driveStrafeInches(double inches, double desiredAngle, double power) throws InterruptedException{
        driveStrafe((int)(inches* DRIVE_STRAFE_ENCODER_TO_INCHES), desiredAngle,power);
    }

    public void driveStrafe(int encoderClicks, double desiredAngle, double power) throws InterruptedException{
        driveStrafe (encoderClicks, desiredAngle,  power, false);
    }

    public void driveStrafe(int encoderClicks, double desiredAngle, double power, boolean useCheat) throws InterruptedException {

        if(useCheat){
            desiredAngle = normalizeAngle(desiredAngle+180);
        }
        float zAngle = 0.0f;
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

        _frontRight.setPower(power);
        _backRight.setPower(-power);
        _frontLeft.setPower(-power);
        _backLeft.setPower(power);

        int averageEncoderClicks = ((Math.abs(_backLeft.getCurrentPosition())+ Math.abs(_frontLeft.getCurrentPosition()))/2);
        //int averageEncoderClicks = _backLeft.getCurrentPosition();
        while ( Math.abs(encoderClicks) > Math.abs(averageEncoderClicks)){
            averageEncoderClicks = Math.abs(_backLeft.getCurrentPosition())+ Math.abs(_frontLeft.getCurrentPosition())/2;
            // information();
            _telemetry.addData(" absolute value of average encoder clicks", Math.abs(averageEncoderClicks));
            _telemetry.update();
            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            if(useCheat) {
                zAngle = (float) normalizeAngle(zAngle + 180);
            }
            // TODO: Issue with angles crossing the circle discontinutiy
            //       e.g. yAngle = 171 and desiredAngle = -179
            //
            double driveCorrect = (zAngle - desiredAngle) * K_TURN;
            double strafeCorrect = (_frontRight.getCurrentPosition()-0.0)*K_STRAFE;  // target is 0


            _frontRight.setPower(power - driveCorrect - strafeCorrect + STRAFE_POWER_OFFSET);
            _backRight.setPower(-power - driveCorrect - strafeCorrect + STRAFE_POWER_OFFSET);
            _frontLeft.setPower(-power + driveCorrect - strafeCorrect + STRAFE_POWER_OFFSET);
            _backLeft.setPower(power + driveCorrect - strafeCorrect + STRAFE_POWER_OFFSET);

            // make +- strafe correct for angle
            // all addition because they all need to turn the same way for strafeCorrect

            //strafe power offset helps even out the drives because the frontLeft and backRigth start too fast and make the robot overcorrect
            //all positives because negative signs are weird and we want those motors to have slightly less power so they don't mov too fast and cause a large correcton

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
        zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;  // 0-90-180-
        // use the cheat?
        if(useCheat){
            zAngle = (float)normalizeAngle(zAngle+180);
            desiredAngle = normalizeAngle(desiredAngle+180);
        }
        // if the angle we want is more than the  angle we are at, spin until you get there
        if (desiredAngle>zAngle) {
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
        while (angleDifference(desiredAngle, zAngle) > 5) {
            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            information();

            if(useCheat) {
                zAngle = (float)normalizeAngle(zAngle + 180);
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

    /***
     * used for finding the
     * @param ang1
     * @param ang2
     * @return
     */
    public double angleDifference(double ang1, double ang2) {
        double a1 = normalizeAngle(ang1);
        double a2 = normalizeAngle(ang2);

        if (Math.abs(a1 - a2) <= 180)
            return Math.abs(a1 - a2);

        if (Math.abs((a1+360) - a2) <= 180)
            return Math.abs((a1+360) - a2);

        return Math.abs(a1 - (a2+360));
    }




    public void information(){
        double convertedClicks = _encoderY.getCurrentPosition()*147.5;
        _telemetry.addData("encoders (inches)",convertedClicks);
        _telemetry.addData("z angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);


        _telemetry.update();
    }

    public void encoderTest(){

        _telemetry.addData("clicks: front left", _frontLeft.getCurrentPosition());
        _telemetry.addData("clicks: front right", _frontRight.getCurrentPosition());
        _telemetry.addData("clicks: back right", _backRight.getCurrentPosition());
        _telemetry.addData("clicks: back left", _backLeft.getCurrentPosition());
        _telemetry.addData("clicks: lifter", _lifter.getCurrentPosition());
        _telemetry.update();
    }

    public void changeLifterPosition(int position){
        if(position!=0) {
            int newTargetPosition = _lifter.getCurrentPosition
                    () + position;
            if (newTargetPosition > 1300) newTargetPosition = 1300;
            if (newTargetPosition < -60) newTargetPosition = -60;
            _lifter.setTargetPosition(newTargetPosition);
            _lifter.setPower(.5);
        }
        _telemetry.addData("lifter target", _lifter.getTargetPosition());
        _telemetry.addData("lifter encoder", _lifter.getCurrentPosition());
        _telemetry.addData("position", position);
        _telemetry.update();
    }

    public void setDriveReadyLifter(){
        _lifter.setTargetPosition(lifterLevel1);
        _lifter.setPower(.5);
    }

    public void setLifterO(){
        _lifter.setTargetPosition(0);
        _lifter.setPower(.5);
    }

    public void setLifterLevel2(){
        _lifter.setTargetPosition(lifterLevel2);
        _lifter.setPower(.5);
    }

    public void setLifterLevel3(){
        _lifter.setTargetPosition(lifterLevel3);
        _lifter.setPower(.5);
    }


    public void duckON(){
        _carouselMover.setPower(duckPower);
    }

    public void duckOFF(){
        _carouselMover.setPower(0);
    }

    public void duckReverse(){
        _carouselMover.setPower(-duckPower);
    }

    @Override
    public void performUpdates() {
        //if strafePower (trigger) is 0 then it will act as a tank drive
        /*
        _frontRight.setPower(_rightPower-_strafePower);
        _backLeft.setPower(_leftPower-_strafePower);
        _frontLeft.setPower(_leftPower+_strafePower);
        _backRight.setPower(_rightPower+_strafePower);

         */


        _frontRight.setPower(_rightPower-_strafePower);
        _backLeft.setPower(_leftPower+_strafePower);
        _frontLeft.setPower(-_rightPower-_strafePower);
        _backRight.setPower(-_leftPower+_strafePower);


        //powers = sticks used to determine what side of the robot the motor is on from collector in the front
        //- means that the motor is corkscrewing backwards

        //strafe right is + due to mechanical front being positive (left is opposite)
    }
}