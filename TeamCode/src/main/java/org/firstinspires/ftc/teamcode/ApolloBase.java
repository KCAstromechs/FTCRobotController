package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class ApolloBase extends AstromechsRobotBase implements TankDriveable, Strafeable {

    //FRONT LEFT IS ENCODER Y, FRONT RIGHT IS ENCODER X

    //Important Set-Up Stuff
    DcMotor _frontLeft;
    DcMotor _backLeft;
    DcMotor _frontRight;
    DcMotor _backRight;
    double _leftPower;
    double _rightPower;
    double _strafePower;
    Telemetry _telemetry;
    final double K_TURN = 0.02;
    BNO055IMU imu;

    //thing that happens when new is used (constructor)
    public ApolloBase(HardwareMap hardwareMap, Telemetry telemetry) {

        //underscore means it's a private variable
        _telemetry = telemetry;
        _frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        _frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        _backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        _backRight = hardwareMap.get(DcMotor.class,"backRight");
        imu = hardwareMap.get(BNO055IMU.class,"imu");


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




        // START THE ENCODERS
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        // GYRO INITIALIZE

        BNO055IMU.Parameters IMUParams = new BNO055IMU.Parameters();IMUParams.mode = BNO055IMU.SensorMode.IMU;
        IMUParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu.initialize(IMUParams);
        while(!imu.isGyroCalibrated());

        float zAngle;
        float yAngle;
        float xAngle;
        zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        xAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;





    }

    public ApolloBase(HardwareMap hardwareMap, Telemetry telemetry, boolean isFC) {

        //underscore means it's a private variable
        _telemetry = telemetry;
        _frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        _frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        _backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        _backRight = hardwareMap.get(DcMotor.class,"backRight");
        imu = hardwareMap.get(BNO055IMU.class,"imu");


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


        // START THE ENCODERS
        _frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // GYRO INITIALIZE

        BNO055IMU.Parameters IMUParams = new BNO055IMU.Parameters();IMUParams.mode = BNO055IMU.SensorMode.IMU;
        IMUParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu.initialize(IMUParams);
        while(!imu.isGyroCalibrated());

        float zAngle;
        float yAngle;
        float xAngle;
        zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        xAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;



    }


//-------------------------------------------------------------------------------------------------------------
//INTERFACE METHODS
//-------------------------------------------------------------------------------------------------------------
    /**
     * sets the power of the left side of the robot
     * @param power will end up being the value from the joysticks in most cases
     */
    @Override
    public void setLeftPower(double power){
        _leftPower = power;
    }

    /**
     * sets the power of the right side of the robot
     * @param power will end up being the value from the joysticks in most cases
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

    /***
     * Will be used to allow strafing in teleop programs
     * @param strafePower equivalent to the "trigger" variable in the mecanum drive
     */
    @Override
    public void strafe(double strafePower){
        _strafePower=strafePower;


    }

//-----------------------------------------------------------------------------------------------------------------
//AUTONOMOUS MOVEMENT
//---------------------------------------------------------------------------------------------------------------

    /***
     * @param inches number of inches you want the robot to move
     * @param desiredAngle given angle you want the robot to move in
     * @param power power you want the robot to make the movement in
     * @throws InterruptedException
     *
     * This particular method uses the other driveStraight methods, but it does the conversion of encoder clicks
     * by multiplying the inches you want by a factor to convert.
     */
    public void driveStraightInches(double inches, double desiredAngle, double power){
        driveStraight((int)(inches*147.5), desiredAngle,power);
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
    public void driveStraight(int encoderClicks, double desiredAngle, double power) {
        driveStraight (encoderClicks, desiredAngle,  power, false);
    }

    /***
     *
     * @param encoderClicks distance desired, measured in encoder clicks
     * @param desiredAngle angle the robot will move in, measured in degrees
     * @param power power the robot will use to move in direction and distance, lower powers are more accurate
     * @param useCheat determine whether or not the robot is trying to move at an angle
     * @throws InterruptedException
     */
    public void driveStraight(int encoderClicks, double desiredAngle, double power, boolean useCheat) {

        //normalizes the angle and effectively moves where the discontinutity is for the purpose of this singlusar movement
        if(useCheat){
            desiredAngle = normalizeAngle(desiredAngle+180);
        }
        //reset the gyro and the motors
        float zAngle = 0.0f;
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(Math.abs(_frontLeft.getCurrentPosition())>10){}


        //set the motors to the desired power, they will keep running during all the logic with the encoders
        _frontRight.setPower(power);
        _backRight.setPower(power);
        _frontLeft.setPower(power);
        _backLeft.setPower(power);


        //checks if the current number of encoder clicks is less than what we want. this will keep running until the current encoder clicks are more than what we want
        while ( Math.abs(encoderClicks) > Math.abs(_frontLeft.getCurrentPosition() )){
            information();
            //normalizes the angle
            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            if(useCheat) {
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

        }
        //turns all the motors off after the desired distance is reached
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
    public void turnToAngle( double desiredAngle, double power ) throws InterruptedException {
        turnToAngle( desiredAngle, power, false );
    }

    // Turn to angle code
    public void turnToAngle( double desiredAngle, double power, boolean useCheat ) throws InterruptedException {

        float zAngle;
        float yAngle;
        float xAngle;

        //follow one motor's encoder count
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(Math.abs(_backLeft.getCurrentPosition())>10){}


        // intialize all of the angles
        zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;  // 0-90-180-
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
            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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

//----------------------------------------------------------------------------------------------------------------------------------
//TELEMETRY AND TROUBLESHOOTING
//-----------------------------------------------------------------------------------------------------------------------------------

    public void information(){
        double convertedClicks = _frontLeft.getCurrentPosition()*147.5;
        _telemetry.addData("encoders (inches)",convertedClicks);
        _telemetry.addData("z angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);


        _telemetry.update();
    }

    public void encoderTest(){

        _telemetry.addData("clicks: ENCODER Y", _frontLeft.getCurrentPosition());
        _telemetry.addData("clicks: encoder X", _frontRight.getCurrentPosition());
        _telemetry.addData("clicks: back right", _backRight.getCurrentPosition());
        _telemetry.addData("clicks: back left", _backLeft.getCurrentPosition());
        //_telemetry.addData("clicks: lifter", _lifter.getCurrentPosition());
        _telemetry.update();
    }

    public void gyroTest(){

        float firstAngle;
        float secondAngle;
        float thirdAngle;
        firstAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        secondAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        thirdAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;


        _telemetry.addData("first angle", firstAngle );
        _telemetry.addData("second angle", secondAngle);
        _telemetry.addData("third angle", thirdAngle);
    }

    public void performUpdates() {
        //if strafePower (trigger) is 0 then it will act as a tank drive
       /*
       _frontRight.setPower(_rightPower-_strafePower);
       _backLeft.setPower(_leftPower-_strafePower);
       _frontLeft.setPower(_leftPower+_strafePower);
       _backRight.setPower(_rightPower+_strafePower);
        */


        _frontRight.setPower(_rightPower+_strafePower);
        _backLeft.setPower(_leftPower+_strafePower);
        _frontLeft.setPower(-_rightPower+_strafePower);
        _backRight.setPower(-_leftPower+_strafePower);



        //powers = sticks used to determine what side of the robot the motor is on from collector in the front
        //- means that the motor is corkscrewing backwards


    }
}

