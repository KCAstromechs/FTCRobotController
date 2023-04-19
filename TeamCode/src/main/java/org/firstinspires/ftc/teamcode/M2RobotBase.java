package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class M2RobotBase extends AstromechsRobotBase implements TankDriveable, Strafeable, FieldCentric {

    //FRONT LEFT IS ENCODER Y, FRONT RIGHT IS ENCODER X
    //type this into command to set up the wifi connect:
    // AppData\Local\Android\Sdk\platform-tools\adb connect 192.168.43.1:5555
    //disconnect from all if having issues
    // AppData\Local\Android\Sdk\platform-tools\adb disconnect


    //Important Set-Up Stuff
    DcMotor _frontLeft;
    DcMotor _backLeft;
    DcMotor _frontRight;
    DcMotor _backRight;
    DcMotor _lifter;
    Servo _rightCollector;
    Servo _leftCollector;
    ColorSensor _colorSensor;
    DistanceSensor _distanceSensor;

    int lifterAutoAdjust = (6*100);
    long endTimeNS = 0;
    boolean slowPingMode = true;
    double lastDistanceReading = 0.0;
    double desiredDistanceFromCone = 2;
    double RIGHT_COLLECTOR_CLOSED = .4;//45
    double RIGHT_COLLECTOR_OPEN = .25;//25
    double LEFT_COLLECTOR_CLOSED = .75; //65
    double LEFT_COLLECTOR_OPEN = .9;
    int lifterZero = 0;
    int DOWN_CORRECT = 500;
    int TOP_SAFETY = 4800;
    int BOTTOM_SAFETY = -200;
    int ZERO_HEIGHT = 0;
    int LOW_HEIGHT = 1850;
    int MID_HEIGHT = 3000;
    int HIGH_HEIGHT = 4000;
    int CONE_STACK_LEVEL_1 = 50;
    int CONE_STACK_LEVEL_2 = 185;
    int CONE_STACK_LEVEL_3 = 350;
    int CONE_STACK_LEVEL_4 = 506;
    int CONE_STACK_LEVEL_5 = 630;
    int minColorDifference = 0;
    double _leftPower;
    double _rightPower;
    double _strafePower;
    double _inputX;
    double _inputY;
    double _turnPower;
    double FCSpeedK = 2;
    Telemetry _telemetry;
    final double K_TURN = 0.03;
    final double K_STRAFE = 0.001;
    public static final double DRIVE_STRAFE_ENCODER_TO_INCHES = 98;
    final double lifterSpeed = 1;
    BNO055IMU imu;
    ElapsedTime _runtime;



    //thing that happens when new is used (constructor)
    public M2RobotBase(HardwareMap hardwareMap, Telemetry telemetry) {

        //underscore means it's a private variable
        _telemetry = telemetry;
        _frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        _frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        _backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        _backRight = hardwareMap.get(DcMotor.class, "backRight");
        _lifter = hardwareMap.get(DcMotor.class, "lifter");
        _rightCollector = hardwareMap.get(Servo.class, "rightCollector");
        _leftCollector = hardwareMap.get(Servo.class, "leftCollector");
        _colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        _distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        _frontRight.setDirection(DcMotor.Direction.REVERSE);
        _backRight.setDirection(DcMotor.Direction.REVERSE);
        _lifter.setDirection(DcMotorSimple.Direction.REVERSE);


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


        // START THE ENCODERS
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _lifter.setTargetPosition(0);
        _lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // GYRO INITIALIZE
        /*


         */
        BNO055IMU.Parameters IMUParams = new BNO055IMU.Parameters();
        IMUParams.mode = BNO055IMU.SensorMode.IMU;
        IMUParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;


        imu.initialize(IMUParams);
        while (!imu.isGyroCalibrated()) ;


    }

    public M2RobotBase(HardwareMap hardwareMap, Telemetry telemetry, boolean isFC) {
        //yeah I know it's redundant i just need to use the variable
        isFC = true;

        //underscore means it's a private variable
        _telemetry = telemetry;
        _frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        _frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        _backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        _backRight = hardwareMap.get(DcMotor.class, "backRight");
        _lifter = hardwareMap.get(DcMotor.class, "lifter");
        _leftCollector = hardwareMap.get(Servo.class, "leftCollector");
        _rightCollector = hardwareMap.get(Servo.class, "rightCollector");
        _colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        _distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        _frontRight.setDirection(DcMotor.Direction.REVERSE);
        _backRight.setDirection(DcMotor.Direction.REVERSE);
        _lifter.setDirection(DcMotorSimple.Direction.REVERSE);


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


        // START THE ENCODERS
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _lifter.setTargetPosition(0);
        _lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        BNO055IMU.Parameters IMUParams = new BNO055IMU.Parameters();
        IMUParams.mode = BNO055IMU.SensorMode.IMU;
        IMUParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu.initialize(IMUParams);
        while (!imu.isGyroCalibrated()) ;


    }


//-------------------------------------------------------------------------------------------------------------
//INTERFACE METHODS
//-------------------------------------------------------------------------------------------------------------

    /**
     * sets the power of the left side of the robot
     *
     * @param power will end up being the value from the joysticks in most cases
     */
    @Override
    public void setLeftPower(double power) {
        _leftPower = power;
    }

    /**
     * sets the power of the right side of the robot
     *
     * @param power will end up being the value from the joysticks in most cases
     */
    @Override
    public void setRightPower(double power) {
        _rightPower = power;
    }

    /**
     * sets power of the left and right sides of the robot
     *
     * @param leftPower
     * @param rightPower
     */
    @Override
    public void setSidePowers(double leftPower, double rightPower) {
        setLeftPower(leftPower);
        setRightPower(rightPower);
    }

    /***
     * Will be used to allow strafing in teleop programs
     * @param strafePower equivalent to the "trigger" variable in the mecanum drive
     */
    @Override
    public void strafe(double strafePower) {
        _strafePower = strafePower;


    }

    public void FCDrive(double inputX, double inputY, double turnPower) {
        _inputX = inputX;
        _inputY = inputY;
        _turnPower = turnPower;

    }

//----------------------------------------------------------------------------------------------------------------
// MECHANISM CONTROL
//-------------------------------------------------------------------------------------------------------------------

    /***
     * puts the collector into the "closed" position, in which it would be holding a cone
     */

    public void collectorClose() {
        _rightCollector.setPosition(RIGHT_COLLECTOR_CLOSED);
        _leftCollector.setPosition(LEFT_COLLECTOR_CLOSED);
    }

    /***
     * puts the collector into the "open" position, in which it would be ready to collect a cone
     */
    public void collectorOpen() {
        _rightCollector.setPosition(RIGHT_COLLECTOR_OPEN);
        _leftCollector.setPosition(LEFT_COLLECTOR_OPEN);

    }

    /***
     * puts the lift into the preset low position
     */

    public void lifterLow() {
        _lifter.setTargetPosition(lifterZero + LOW_HEIGHT);
        _lifter.setPower(lifterSpeed);
    }

    /***
     * puts the lift into the preset medium position
     */
    public void lifterMedium() {
        _lifter.setTargetPosition(lifterZero + MID_HEIGHT);
        _lifter.setPower(lifterSpeed);
    }

    /***
     * puts the lift into the preset high position
     */
    public void lifterHigh() {
        _lifter.setTargetPosition(lifterZero + HIGH_HEIGHT);
        _lifter.setPower(lifterSpeed);

    }
    /***
     * lowers the lifter a preset amount
     */
    public void scootLifterDown() {
        _lifter.setTargetPosition(_lifter.getCurrentPosition() - DOWN_CORRECT);
        _lifter.setPower(lifterSpeed);
    }
    /***
     * brings the lifter up a preset amount
     */
    public void scootLifterUp() {
        _lifter.setTargetPosition(_lifter.getCurrentPosition() + DOWN_CORRECT);
        _lifter.setPower(lifterSpeed);
    }
    /***
     * puts the lift into position for the fourth cone in the stack
     */
    public void lifterCS4() {
        _lifter.setTargetPosition(lifterZero + CONE_STACK_LEVEL_4);
        _lifter.setPower(lifterSpeed);

    }
    /***
     * puts the lift into position for the third cone in the stack
     */

    public void lifterCS3() {
        _lifter.setTargetPosition(lifterZero + CONE_STACK_LEVEL_3);
        _lifter.setPower(lifterSpeed);

    }
    /***
     * puts the lift into position for the second cone in the stack
     */
    public void lifterCS2() {
        _lifter.setTargetPosition(lifterZero + CONE_STACK_LEVEL_2);
        _lifter.setPower(lifterSpeed);

    }
    /***
     * puts the lift into position for the first cone in the stack
     */

    public void lifterCS1() {
        _lifter.setTargetPosition(lifterZero + CONE_STACK_LEVEL_1);
        _lifter.setPower(lifterSpeed);

    }
    /***
     * puts the lift into position for the fifth cone in the stack
     */

    public void lifterCS5() {
        _lifter.setTargetPosition(lifterZero + CONE_STACK_LEVEL_5);
        _lifter.setPower(lifterSpeed);

    }

    /***
     * puts the lift in its lowest point
     */
    public void lifterZero() {
        _lifter.setTargetPosition(lifterZero + ZERO_HEIGHT);
        _lifter.setPower(lifterSpeed);
    }


    /***
     * allows for the teleOp control of the lifter by taking in the joystick y value
     * @param position the value incoming from a y joystick value that adds/subtracts to the height of the lifter
     */
    public void lifterControl(int position) {
        if (position != 0) {
            int newTargetPosition = _lifter.getCurrentPosition() + position;
            _lifter.setPower(1);

            if (newTargetPosition > TOP_SAFETY)
                newTargetPosition = TOP_SAFETY;
            _lifter.setTargetPosition(newTargetPosition);
            _lifter.setPower(1);

                /*
            if(newTargetPosition < BOTTOM_SAFETY)
                 newTargetPosition = BOTTOM_SAFETY;
                _lifter.setTargetPosition(newTargetPosition);
                _lifter.setPower(1);

                 */
        }
        // data time and safety encoder checks
        double lifterTarget =  _lifter.getTargetPosition();
        double lifterCurrent = _lifter.getCurrentPosition();
        _telemetry.addData("lifter target", lifterTarget);
        _telemetry.addData("lifter encoder", lifterCurrent);
        _telemetry.addData("position", position);
        if ((lifterCurrent < 0) && (lifterCurrent < lifterZero)) {
            lifterZero = (int)lifterCurrent + 30;
            _telemetry.addData("lifterZero has been reset", "yay");
        }
        _telemetry.update();
    }

    public void lifterResetDown() throws InterruptedException {
        _lifter.setPower(-.2);
        Thread.sleep(500);
        _lifter.setPower(0);
        _lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Thread.sleep(20);
        _lifter.setTargetPosition(0);
        _lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void lifterResetUp() throws InterruptedException {
        _lifter.setPower(.2);
        Thread.sleep(500);
        _lifter.setPower(0);
        _lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Thread.sleep(20);
        _lifter.setTargetPosition(0);
        _lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public boolean isClosed(){
        if ((_leftCollector.getPosition() >= .35) && (_rightCollector.getPosition() >= .35)){
            return true;
        }
        else {
            return false;
        }
    }

//-----------------------------------------------------------------------------------------------------------------
//AUTONOMOUS MOVEMENT
//---------------------------------------------------------------------------------------------------------------

    public void motorShutdown(){
        _frontRight.setPower(0);
        _frontLeft.setPower(0);
        _backRight.setPower(0);
        _backLeft.setPower(0);
        _lifter.setPower(0);
    }

    /***
     * @param inches number of inches you want the robot to move
     * @param desiredAngle given angle you want the robot to move in
     * @param power power you want the robot to make the movement in
     * @throws InterruptedException
     *
     * This particular method uses the other driveStraight methods, but it does the conversion of encoder clicks
     * by multiplying the inches you want by a factor to convert.
     */
    public void driveStraightInches(double inches, double desiredAngle, double power,
    long timeLimitMS) throws DriveTimeoutException {
        driveStraight((int) (inches * 147.5), desiredAngle, power, timeLimitMS);
    }

    /***
     *
     * @param encoderClicks number of encoder clicks the robot will move
     * @param desiredAngle angle the robot will move in
     * @param power power the robot will use to move in given angle and direction
     * @throws InterruptedException
     *
     * runs drivestraight in the cases that the robot is not using a degree measurement that falls within the angle discontinuity
     */
    public void driveStraight(int encoderClicks, double desiredAngle, double power,
    long timeLimitMs) throws DriveTimeoutException {
        driveStraight(encoderClicks, desiredAngle, power, false, timeLimitMs);
    }

    /***
     *
     * @param encoderClicks distance desired, measured in encoder clicks
     * @param desiredAngle angle the robot will move in, measured in degrees
     * @param power power the robot will use to move in direction and distance, lower powers are more accurate
     * @param useCheat determine whether or not the robot is trying to move at an angle
     * @throws InterruptedException
     */


    public void driveStraight(int encoderClicks, double desiredAngle, double power, boolean useCheat,
      long timeLimitMS) throws DriveTimeoutException {

        long endTimeNS = (System.nanoTime() + (timeLimitMS*1000000L));

        //normalizes the angle and effectively moves where the discontinutity is for the purpose of this singlusar movement
        if (useCheat) {
            desiredAngle = normalizeAngle(desiredAngle + 180);
        }
        //reset the gyro and the motors
        float zAngle = 0.0f;
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(_frontLeft.getCurrentPosition()) > 50) {
        }
// was 10

        //set the motors to the desired power, they will keep running during all the logic with the encoders
        _frontRight.setPower(power);
        _backRight.setPower(power);
        _frontLeft.setPower(power);
        _backLeft.setPower(power);


        //checks if the current number of encoder clicks is less than what we want. this will keep running until the current encoder clicks are more than what we want
        while (Math.abs(encoderClicks) > Math.abs(_frontLeft.getCurrentPosition())) {
            //normalizes the angle
            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if (useCheat) {
                zAngle = (float) normalizeAngle(zAngle + 180);
            }
            // TODO: Issue with angles crossing the circle discontinutiy
            //       e.g. yAngle = 171 and desiredAngle = -179


            //drivecorrect finds the number of degrees we are off from the current course we want, and then multiples it by a factor that is robot specific, which will allow the robot to move back to the desired angle
            double driveCorrect = (zAngle - desiredAngle) * K_TURN;
            _frontRight.setPower(power - driveCorrect);
            _backRight.setPower(power - driveCorrect);
            _frontLeft.setPower(power + driveCorrect);
            _backLeft.setPower(power + driveCorrect);

            if (System.nanoTime() > endTimeNS){
                throw new DriveTimeoutException();
            }

        }
        //turns all the motors off after the desired distance is reached
        _frontRight.setPower(0);
        _backRight.setPower(0);
        _frontLeft.setPower(0);
        _backLeft.setPower(0);
    }

    public void driveStrafeInches(double inches, double desiredAngle, double power, long timeLimitMS) throws InterruptedException, DriveTimeoutException {
        driveStrafe((int) (inches * DRIVE_STRAFE_ENCODER_TO_INCHES), desiredAngle, power, timeLimitMS);
    }

    public void driveStrafe(int encoderClicks, double desiredAngle, double power, long timeLimitMS) throws InterruptedException, DriveTimeoutException {
        driveStrafe(encoderClicks, desiredAngle, power, false, timeLimitMS);
    }

    public void driveStrafe(int encoderClicks, double desiredAngle, double power, boolean useCheat,
    long timeLimitMS) throws InterruptedException, DriveTimeoutException {
       long endTimeNS = (System.nanoTime() + (timeLimitMS*1000000L));


        //throw new dte();

        if (useCheat) {
            desiredAngle = normalizeAngle(desiredAngle + 180);
        }
        float zAngle = 0.0f;
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

        _frontRight.setPower(power);
        _backRight.setPower(-power);
        _frontLeft.setPower(-power);
        _backLeft.setPower(power);


        while (Math.abs(_frontRight.getCurrentPosition()) < Math.abs(encoderClicks)) {
            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if (useCheat) {
                zAngle = (float) normalizeAngle(zAngle + 180);
            }
            double driveCorrect = (zAngle - desiredAngle) * K_TURN;
            double strafeCorrect = (-_frontLeft.getCurrentPosition() - 0.0) * K_STRAFE;  // target is 0, finds the change in the y encoder value and converts it to a power value to modify the power parameter. Subracts zero because that's the number we want it to be at


            _frontRight.setPower(power - driveCorrect - strafeCorrect);
            _backRight.setPower(-power - driveCorrect - strafeCorrect);
            _frontLeft.setPower(-power + driveCorrect - strafeCorrect);
            _backLeft.setPower(power + driveCorrect - strafeCorrect);
            information();

            if (System.nanoTime() > endTimeNS){
                throw new DriveTimeoutException();
            }

            // make +- strafe correct for angle
            // all addition because they all need to turn the same way for strafeCorrect



        }

        _frontRight.setPower(0);
        _backRight.setPower(0);
        _frontLeft.setPower(0);
        _backLeft.setPower(0);
    }


    /***
     * @param desiredAngle angle you want the front of the robot to face at the end of the turn
     * @param power power you want the robot to use to make the turn, lower powers are more precise
     * @throws InterruptedException
     */
    public void turnToAngle(double desiredAngle, double power) throws InterruptedException {
        turnToAngle(desiredAngle, power, false);
    }

    // Turn to angle code
    public void turnToAngle(double desiredAngle, double power, boolean useCheat) throws InterruptedException {

        float zAngle;
        float yAngle;
        float xAngle;

        //follow one motor's encoder count
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(_backLeft.getCurrentPosition()) > 10) {
        }


        // intialize all of the angles
        zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;  // 0-90-180-
        // use the cheat?
        if (useCheat) {
            zAngle = (float) normalizeAngle(zAngle + 180);
            desiredAngle = normalizeAngle(desiredAngle + 180);
        }
        // if the angle we want is more than the  angle we are at, spin until you get there
        if (desiredAngle > zAngle) {
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
            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if (useCheat) {
                zAngle = (float) normalizeAngle(zAngle + 180);
            }
            sleep(10);
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
            a = a - 360;
        while (a < 0.0)
            a = a + 360;
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

        if (Math.abs((a1 + 360) - a2) <= 180)
            return Math.abs((a1 + 360) - a2);

        return Math.abs(a1 - (a2 + 360));
    }

    public double getDistanceReading(){

        if (slowPingMode) {
            if (System.nanoTime() > endTimeNS) {
                lastDistanceReading = _distanceSensor.getDistance(DistanceUnit.CM);
               // _distanceSensor.close();
                //50 is the amount of time we want to wait in between asking the question
                endTimeNS = (System.nanoTime() + (50 * 1000000L));
            }
            return lastDistanceReading;
        }
        return _distanceSensor.getDistance(DistanceUnit.CM);
    }

    public void setDistanceSLowMode(boolean _s){
        slowPingMode =_s;
    }

    public long distanceSensorDelayTest(){
        long startTime = System.nanoTime();
        long timeAfterAnswer = 0;
        _distanceSensor.getDistance(DistanceUnit.CM);
        timeAfterAnswer = System.nanoTime();
        return (startTime - timeAfterAnswer);

    }


    public void horizontalJunctionDistanceDetect(boolean moveLeft, float desiredAngle, int desiredClicks, double power) throws InterruptedException {
        //NEVER EVER NEVER EVER NEVER EVER MAKE THE POWER NEGATIVE IT'LL MESS EVERYTHING UP
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

//15
            while ( (Math.abs(getDistanceReading() - 4.5) > 20) && (Math.abs(_frontRight.getCurrentPosition()) < desiredClicks)) {
                float zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double driveCorrect = (zAngle - desiredAngle) * K_TURN;
                double strafeCorrect = (-_frontLeft.getCurrentPosition() - 0.0) * K_STRAFE;  // target is 0, finds the chan

                _telemetry.addData("HORIZONTAL distance", getDistanceReading());
                _telemetry.update();


                //strafe over


                if (moveLeft) {
                    _frontLeft.setPower(-power + driveCorrect - strafeCorrect);
                    _frontRight.setPower(power - driveCorrect - strafeCorrect);
                    _backLeft.setPower(power + driveCorrect - strafeCorrect);
                    _backRight.setPower(-power - driveCorrect - strafeCorrect);
                }
                else{
                    _frontLeft.setPower(power + driveCorrect - strafeCorrect);
                    _frontRight.setPower(-power - driveCorrect - strafeCorrect);
                    _backLeft.setPower(-power + driveCorrect - strafeCorrect);
                    _backRight.setPower(power - driveCorrect - strafeCorrect);

                }



            }
                _frontLeft.setPower(0);
                _frontRight.setPower(0);
                _backLeft.setPower(0);
                _backRight.setPower(0);
                _telemetry.clear();

            }


    public void forwardJunctionDistanceDetect(int desiredClicks) throws InterruptedException {

        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);


        while (((Math.abs(getDistanceReading() - 4.5) > 5) && ((Math.abs(getDistanceReading() - 4.5) < 25))) && (Math.abs(_frontRight.getCurrentPosition()) < desiredClicks))  {

             _telemetry.addData("currentDistance", getDistanceReading());
             _telemetry.update();

                _frontLeft.setPower(.2);
                _frontRight.setPower(.2);
                _backLeft.setPower(.2);
                _backRight.setPower(.2);

        }
        _frontLeft.setPower(0);
        _frontRight.setPower(0);
        _backLeft.setPower(0);
        _backRight.setPower(0);
    }





    public boolean colorSensorDetect(boolean isBlueTape, float desiredAngle, boolean moveLeft, int desiredClicks) throws InterruptedException {

        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

        //if we are on the blue side
        if (isBlueTape) {
            //while the color sensor does not read within 80-100
            while (_colorSensor.blue() < _colorSensor.green() + minColorDifference && (Math.abs(_frontRight.getCurrentPosition()) < desiredClicks)) {
                float zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double driveCorrect = (zAngle - desiredAngle) * K_TURN;
                double strafeCorrect = (-_frontLeft.getCurrentPosition() - 0.0) * K_STRAFE;  // target is 0, finds the chan
                //strafe over
                _telemetry.addData("blue strength", _colorSensor.blue() - _colorSensor.green());
                _telemetry.update();


                if (moveLeft) {
                    _frontLeft.setPower(-.3 + driveCorrect - strafeCorrect);
                    _frontRight.setPower(.3 - driveCorrect - strafeCorrect);
                    _backLeft.setPower(.3 + driveCorrect - strafeCorrect);
                    _backRight.setPower(-.3 - driveCorrect - strafeCorrect);
                }
                else{
                    _frontLeft.setPower(.3 + driveCorrect - strafeCorrect);
                    _frontRight.setPower(-.3 - driveCorrect - strafeCorrect);
                    _backLeft.setPower(-.3 + driveCorrect - strafeCorrect);
                    _backRight.setPower(.3 - driveCorrect - strafeCorrect);

                }



            }
            //if the proper color is read
            //turn the power off and tell the program that the drive was successful
            if (_colorSensor.blue() > _colorSensor.green() + minColorDifference) {
                _telemetry.addData("SUCCESSS!!!", "Color go brrrrrr");
                _telemetry.update();
                _frontLeft.setPower(0);
                _frontRight.setPower(0);
                _backLeft.setPower(0);
                _backRight.setPower(0);
                return true;
            }
        }
        else {
            //while the color sensor does not read within 80-100
            while (_colorSensor.red() < _colorSensor.green() + minColorDifference && (Math.abs(_frontRight.getCurrentPosition()) < desiredClicks)) {
                float zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double driveCorrect = (zAngle - desiredAngle) * K_TURN;
                double strafeCorrect = (-_frontLeft.getCurrentPosition() - 0.0) * K_STRAFE;  // target is 0, finds the chan
                //strafe over
                _telemetry.addData("red strength", _colorSensor.red() - _colorSensor.green());
                _telemetry.update();

                if (moveLeft) {
                    _frontLeft.setPower(-.3 + driveCorrect - strafeCorrect);
                    _frontRight.setPower(.3 - driveCorrect - strafeCorrect);
                    _backLeft.setPower(.3 + driveCorrect - strafeCorrect);
                    _backRight.setPower(-.3 - driveCorrect - strafeCorrect);
                }
                else{
                    _frontLeft.setPower(.3 + driveCorrect - strafeCorrect);
                    _frontRight.setPower(-.3 - driveCorrect - strafeCorrect);
                    _backLeft.setPower(-.3 + driveCorrect - strafeCorrect);
                    _backRight.setPower(.3 - driveCorrect - strafeCorrect);

                }
            }
            //if the proper color is read
            //turn the power off and tell the program that the drive was successful
            if (_colorSensor.red() > _colorSensor.green() + minColorDifference) {
                _telemetry.addData("SUCCESSS!!!", "Color go brrrrrr");
                _telemetry.update();
                _frontLeft.setPower(0);
                _frontRight.setPower(0);
                _backLeft.setPower(0);
                _backRight.setPower(0);
                _colorSensor.enableLed(false);
                return true;
            }
        }

        //if the robot has moved too far as a safety
        //turn power off an report that the drive was unsuccessful
        _telemetry.addData("FAILED", "ENCODER DRIVE");
        _telemetry.update();
        _frontLeft.setPower(0);
        _frontRight.setPower(0);
        _backLeft.setPower(0);
        _backRight.setPower(0);
        _colorSensor.enableLed(false);
        return false;
    }

    public boolean coneDrive(double coneDriveMaxDistanceInches, float desiredAngle, double power) throws InterruptedException {

        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(250);

        while (Math.abs(_frontLeft.getCurrentPosition()) < (coneDriveMaxDistanceInches*147.5) && getDistanceReading() - 4.5 >= desiredDistanceFromCone){
            float zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double driveCorrect = (zAngle - desiredAngle) * K_TURN;
            double strafeCorrect = (-_frontLeft.getCurrentPosition() - 0.0) * K_STRAFE;  // target is 0, finds the chan
            //_telemetry.addData("status", "seeking");

            _frontRight.setPower(power - driveCorrect);
            _backRight.setPower(power - driveCorrect);
            _frontLeft.setPower(power + driveCorrect);
            _backLeft.setPower(power + driveCorrect);
        }
        // stop moving
        _frontLeft.setPower(0);
        _frontRight.setPower(0);
        _backLeft.setPower(0);
        _backRight.setPower(0);

        // did we hit max distance or see the cone?
        if (getDistanceReading() - 4.5 <= 9) {
            _telemetry.addData("status","detected");
            _telemetry.update();
            _colorSensor.enableLed(false);
            return true;
        }
        else {
            _telemetry.addData("status","distance exceeded");
            _telemetry.update();
            _colorSensor.enableLed(false);
            return false;
        }
    }







//----------------------------------------------------------------------------------------------------------------------------------
//TELEMETRY AND TROUBLESHOOTING
//-----------------------------------------------------------------------------------------------------------------------------------

    public void frontLeftTest(){
       _frontLeft.setPower(.4);
    }
    public void backLeftTest(){
        _backLeft.setPower(.4);
    }
    public void frontRightTest(){
        _frontRight.setPower(.4);
    }
    public void backRightTest(){
        _backRight.setPower(.4);
    }

    public void turnColorLEDOff(){
        _colorSensor.enableLed(false);
    }

    public void information() {
        double convertedClicks = _frontLeft.getCurrentPosition() * 147.5;
        _telemetry.addData("encoders (inches)", convertedClicks);
        _telemetry.addData("z angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);


        _telemetry.update();
    }

    public void encoderTest() {

        _telemetry.addData("clicks: ENCODER Y", _frontLeft.getCurrentPosition());
        _telemetry.addData("clicks: encoder X", _frontRight.getCurrentPosition());
        _telemetry.addData("clicks: back right", _backRight.getCurrentPosition());
        _telemetry.addData("clicks: back left", _backLeft.getCurrentPosition());
        _telemetry.addData("clicks: lifter", _lifter.getCurrentPosition());
        _telemetry.update();
    }

    public void gyroTest() {

        float firstAngle;
        float secondAngle;
        float thirdAngle;
        firstAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        secondAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        thirdAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;


        _telemetry.addData("first angle", firstAngle);
        _telemetry.addData("second angle", secondAngle);
        _telemetry.addData("third angle", thirdAngle);
    }

    public void performUpdates() {
        //if strafePower (trigger) is 0 then it will act as a tank drive

        _frontRight.setPower(_rightPower - _strafePower);
        _backLeft.setPower(_leftPower - _strafePower);
        _frontLeft.setPower(_leftPower + _strafePower);
        _backRight.setPower(_rightPower + _strafePower);


/*
       _frontRight.setPower(_rightPower-_strafePower);
       _backLeft.setPower(_leftPower-_strafePower);
       _frontLeft.setPower(-_rightPower-_strafePower);
       _backRight.setPower(-_leftPower-_strafePower);

*/


/*
        _frontRight.setPower(-_leftPower-_strafePower);
        _frontLeft.setPower(_leftPower-_strafePower);
        _backRight.setPower(_rightPower-_strafePower);
        _backLeft.setPower(-_rightPower-_strafePower);

 */


        //powers = sticks used to determine what side of the robot the motor is on from collector in the fro//- means that the motor is corkscrewing backwards


    }

    /**
     * TODO
     * @param angleOffset
     * @param UseSlowMode
     * @param autoGrab
     */
    public void performFCUpdates(double angleOffset, boolean UseSlowMode, boolean autoGrab) throws  InterruptedException {
        double zAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - angleOffset;
        double K = FCSpeedK; // if you want to create a slow mode, replace or change K

        double fRPower;
        double fLPower;
        double bRPower;
        double bLPower;

        double robotX = (_inputX * Math.sin(zAngle)) + (_inputY * Math.sin(zAngle + (PI / 2)));
        double robotY = ((_inputX * Math.cos(zAngle)) + (_inputY * Math.cos(zAngle + (PI / 2)))) * 1.4;



        if (autoGrab && (getDistanceReading() - 6.0) <= 0) {
            _telemetry.addData("distance", (getDistanceReading()));
            _telemetry.update();
            // MAKE SURE ROBOT IS SAFE DURING THREAD.SLEEP
            _inputX = 0;
            _inputY = 0;
            _turnPower = 0;
            _frontLeft.setPower(0);
            _frontRight.setPower(0);
            _backRight.setPower(0);
            _backLeft.setPower(0);
            // close the collector
            collectorClose();
            // wait for collector to close
            Thread.sleep(350);
            // lift slightly above cone stack or ground
            _lifter.setPower(.5);
            _lifter.setTargetPosition((_lifter.getCurrentPosition() +lifterAutoAdjust));
        }



        if (UseSlowMode || (_lifter.getCurrentPosition() > MID_HEIGHT-200)) {
            _turnPower *= 1.25;
             fLPower = ((robotX + robotY) + _turnPower) / K;
             fRPower = ((robotX - robotY) - _turnPower) / K;
             bRPower = ((robotX + robotY) - _turnPower) / K;
             bLPower = ((robotX - robotY) + _turnPower) / K;
        }
        else {
             fLPower = ((robotX + robotY) + _turnPower);
             fRPower = ((robotX - robotY) - _turnPower);
             bRPower = ((robotX + robotY) - _turnPower);
             bLPower = ((robotX - robotY) + _turnPower);

        }

        double highestPowerF = Math.max(Math.abs(fLPower), Math.abs(fRPower));
        double highestPowerB = Math.max(Math.abs(bRPower), Math.abs(bLPower));
        double highestPower = Math.max(highestPowerB, highestPowerF);

        if (highestPower > 1) {
            fLPower /= highestPower;
            fRPower /= highestPower;
            bRPower /= highestPower;
            bLPower /= highestPower;
        }

        _frontLeft.setPower(fLPower);
        _frontRight.setPower(fRPower);
        _backRight.setPower(bRPower);
        _backLeft.setPower(bLPower);

    }
}


