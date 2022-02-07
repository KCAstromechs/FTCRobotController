
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ApolloBase implements TankDriveable, Strafeable {

    //Important Set-Up Stuff
    DcMotor _frontLeft;
    DcMotor _backLeft;
    DcMotor _frontRight;
    DcMotor _backRight;

    double _leftPower;
    double _rightPower;
    double _strafePower;



    public ApolloBase(HardwareMap hardwareMap) {
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
    public void strafe(double strafePower) {
        _strafePower = strafePower;

    }
    public void performUpdates() {


        _frontRight.setPower(_rightPower+_strafePower);
        _backLeft.setPower(_leftPower+_strafePower);
        _frontLeft.setPower(-_rightPower+_strafePower);
        _backRight.setPower(-_leftPower+_strafePower);


        //powers = sticks used to determine what side of the robot the motor is on from collector in the front
        //- means that the motor is corkscrewing backwards

        //strafe right is + due to mechanical front being positive (left is opposite)

    }
}