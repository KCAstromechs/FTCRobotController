
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class M2_Robot_Base extends AstromechsRobotBase implements TankDriveable, Strafeable {

    public static final double duckPower = .50;
    public static final double DRIVE_STRAIGHT_ENCODER_TO_INCHES = 118;
    public static final double DRIVE_STRAFE_ENCODER_TO_INCHES = 67;
    final double autoDuckPower = .25;
    //Important Set-Up Stuff
    DcMotor _frontLeft;
    DcMotor _backLeft;
    DcMotor _frontRight;
    DcMotor _backRight;
    DcMotor _backSpinner;
    DcMotor _frontSpinner;
    DcMotor _intake;
    DcMotor _lifter;
    Servo _capper;
    double _leftPower;
    double _rightPower;
    double _strafePower;
    double _lifterPower;
    double _intakePower;
    Telemetry _telemetry;
    final double K_TURN = 0.02;
    final double K_STRAFE = 0.02;
    BNO055IMU imu;
    final int lifterLevel1 =500;
    final int lifterLevel2 =773;
    final int lifterLevel3 =1365;
    final int testDistance = 2000;
    final int testDistanceWheels = 10;


    //thing that happens when new is used (constructor)
    public M2_Robot_Base(HardwareMap hardwareMap, Telemetry telemetry) {

        //underscore means it's a private variable
        _telemetry = telemetry;
        _capper = hardwareMap.get(Servo.class, "capper");
        _frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        _frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        _backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        _backRight = hardwareMap.get(DcMotor.class,"backRight");
        _frontSpinner = hardwareMap.get(DcMotor.class, "frontSpinner");
        _backSpinner = hardwareMap.get(DcMotor.class, "backSpinner");
        _lifter= hardwareMap.get(DcMotor.class, "lifter");
        _intake = hardwareMap.get(DcMotor.class, "intake");
        imu = hardwareMap.get(BNO055IMU.class,"imu");


        _frontRight.setDirection(DcMotor.Direction.REVERSE);
        _backRight.setDirection(DcMotor.Direction.REVERSE);
        _lifter.setDirection(DcMotor.Direction.REVERSE);
        _backSpinner.setDirection(DcMotor.Direction.REVERSE);
        _lifter.setDirection(DcMotor.Direction.REVERSE);
        _intake.setDirection(DcMotor.Direction.REVERSE);

        _backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // RESET ENCODERS
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // START THE ENCODERS
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        driveStraight((int)(inches* DRIVE_STRAIGHT_ENCODER_TO_INCHES), desiredAngle,power);
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
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

          _frontRight.setPower(power);
          _backRight.setPower(power);
          _frontLeft.setPower(power);
          _backLeft.setPower(power);

        while ( Math.abs(encoderClicks) > Math.abs(_frontRight.getCurrentPosition() )){
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


            _frontRight.setPower(power - driveCorrect);
            _backRight.setPower(-power - driveCorrect);
            _frontLeft.setPower(-power + driveCorrect);
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

    /***
     *@param robotPower
     * will always be either .1 or -.1
     * needed because we have two spinners
     *
     *
     *
     * @param isBlue
     * is used for determining whether the motor power should be positive or negative
     * true is for blue side autos
     * false is for red side autos
     */
    public void deliverDuck(boolean isBlue, double robotPower){
        _frontSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(Math.abs(_frontSpinner.getCurrentPosition())<3000 || Math.abs(_backSpinner.getCurrentPosition())<3000){
            _telemetry.addData("front spinner", _frontSpinner.getCurrentPosition());
            _telemetry.addData("back spinner", _backSpinner.getCurrentPosition());
            _telemetry.update();

            if (isBlue == true) {
                _frontSpinner.setPower(-autoDuckPower);
                _backSpinner.setPower(autoDuckPower);
            }
            else{
                _frontSpinner.setPower(autoDuckPower);
                _backSpinner.setPower(-autoDuckPower);
            }
            _backLeft.setPower(robotPower);
            _frontLeft.setPower(robotPower);
            _backRight.setPower(robotPower);
            _frontRight.setPower(robotPower);
        }
        _backLeft.setPower(0);
        _frontLeft.setPower(0);
        _backRight.setPower(0);
        _frontRight.setPower(0);
        _frontSpinner.setPower(0);
        _backSpinner.setPower(0);
    }

    public void information(){
        double convertedClicks = _frontRight.getCurrentPosition()* DRIVE_STRAFE_ENCODER_TO_INCHES;
        _telemetry.addData("encoders (inches)",convertedClicks);
        _telemetry.addData("z angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);_telemetry.addData("front right power", _frontRight.getPower());
        _telemetry.addData("back right power", _backRight.getPower());
        _telemetry.addData("front left power", _frontLeft.getPower());
        _telemetry.addData("back left power", _backLeft.getPower());
        _telemetry.addData("front right power", _frontRight.getCurrentPosition());



        _telemetry.update();
    }

    public void encoderTest(){
        _telemetry.addData("clicks: front spinner", _frontSpinner.getCurrentPosition());
        _telemetry.addData("clicks: back spinner", _backSpinner.getCurrentPosition());
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

    public void setLifterLevel2(){
        _lifter.setTargetPosition(lifterLevel2);
        _lifter.setPower(.5);
    }
    public void setLifterLevel3(){
        _lifter.setTargetPosition(lifterLevel3);
        _lifter.setPower(.5);
    }


    public void setLifterO(){
        _lifter.setTargetPosition(0);
        _lifter.setPower(.5);
    }

    public void setIntakeCollect(){
        _intakePower = .8;
        _intake.setPower(_intakePower);
    }

    public void setIntakeDischarge(){
        _intakePower = -.8;
        _intake.setPower(_intakePower);
    }

    public void setIntakeOff(){
        _intakePower = 0;
        _intake.setPower(_intakePower);
    }

    public void duckON(){
        _frontSpinner.setPower(duckPower);
        _backSpinner.setPower(-duckPower);
    }

    public void duckOFF(){
        _frontSpinner.setPower(0);
        _backSpinner.setPower(0);
    }

    public void duckReverse(){
        _frontSpinner.setPower(-duckPower);
        _backSpinner.setPower(-duckPower);
    }

    public void setCapperDelivered(){
        _capper.setPosition(.97);
    }

    public void setCapperUndelivered(){
        _capper.setPosition(.5);
    }
    @Override
    public void performUpdates() {


        _frontRight.setPower(_rightPower+_strafePower);
        _backLeft.setPower(_leftPower+_strafePower);
        _frontLeft.setPower(-_rightPower+_strafePower);
        _backRight.setPower(-_leftPower+_strafePower);


        //powers = sticks used to determine what side of the robot the motor is on from collector in the front
        //- means that the motor is corkscrewing backwards

        //strafe right is + due to mechanical front being positive (left is opposite)

    }
/*--------------------------------------------------------------------------------------------------------------------------------------------------------
   TEST FUNCTIONS
----------------------------------------------------------------------------------------------------------------------------------------------------------
*/
    public void TestSetIntakeCollect(){
        _intakePower = .8;
        while(Math.abs(_intake.getCurrentPosition())>testDistance) {
            _intake.setPower(_intakePower);
            _telemetry.addData("clicks", _intake.getCurrentPosition());
            _telemetry.addData("intake function:", "collecting");
            _telemetry.update();
        }
        _intake.setPower(0);
    }

    public void TestSetIntakeDischarge(){
        _intakePower = -.8;
        while(Math.abs(_intake.getCurrentPosition())>testDistance) {
            _intake.setPower(-_intakePower);
            _telemetry.addData("clicks", _intake.getCurrentPosition());
            _telemetry.addData("intake function:", "delivering/outake");
            _telemetry.update();
        }
        _intake.setPower(0);

    }

    public void TestSetIntakeOff(){
        _intakePower = 0;
        _intake.setPower(_intakePower);
    }

    public void TestDuckON(){
        while(Math.abs(_frontSpinner.getCurrentPosition())>testDistance) {
            _frontSpinner.setPower(duckPower);
            _backSpinner.setPower(-duckPower);

            _telemetry.addData(" right wheel clicks", _frontSpinner.getCurrentPosition());
            _telemetry.addData("left wheel clicks", _backSpinner.getCurrentPosition());
            _telemetry.addData("carousel function:", "BLUE SIDE");
            _telemetry.update();
        }
        _frontSpinner.setPower(0);
        _backSpinner.setPower(0);

    }

    public void TestDuckOFF(){

        _frontSpinner.setPower(0);
        _backSpinner.setPower(0);
    }

    public void TestDuckReverse(){
        while(Math.abs(_frontSpinner.getCurrentPosition())>testDistance) {
            _frontSpinner.setPower(-duckPower);
            _backSpinner.setPower(duckPower);

            _telemetry.addData(" right wheel clicks", _frontSpinner.getCurrentPosition());
            _telemetry.addData("left wheel clicks", _backSpinner.getCurrentPosition());
            _telemetry.addData("carousel function:", "RED SIDE");
            _telemetry.update();
        }
        _frontSpinner.setPower(0);
        _backSpinner.setPower(0);

    }

    public void backLeftTest(){
        while(Math.abs(_backLeft.getCurrentPosition())>(testDistanceWheels*DRIVE_STRAIGHT_ENCODER_TO_INCHES)){
            _backLeft.setPower(.4);
        }
        _backLeft.setPower(0);
    }

    public void frontLeftTest(){
        while(Math.abs(_frontLeft.getCurrentPosition())>(testDistanceWheels*DRIVE_STRAIGHT_ENCODER_TO_INCHES)){
            _frontLeft.setPower(.4);
        }
        _frontLeft.setPower(0);
    }

    public void backRightTest(){
        while(Math.abs(_backRight.getCurrentPosition())>(testDistanceWheels*DRIVE_STRAIGHT_ENCODER_TO_INCHES)){
            _backRight.setPower(.4);
        }
        _backRight.setPower(0);
    }

    public void frontRightTest(){
        while(Math.abs(_frontRight.getCurrentPosition())>(testDistanceWheels*DRIVE_STRAIGHT_ENCODER_TO_INCHES)){
            _frontRight.setPower(.4);
        }
        _frontRight.setPower(0);
    }










}



