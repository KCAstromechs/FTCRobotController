package org.firstinspires.ftc.teamcode;

/**
 * used for robots that utilize a tank drive.
 */
public interface TankDriveable {

    /**
     * sets the power of the left side of the robot
     * @param power number used for setting the power
     */
    void setLeftPower(double power);

    /**
     * sets the power of the right side of the robot
     * @param power number used for setting the power
     */
    void setRightPower(double power);

    /**
     * used to set the power on both sides of the robot
     * @param leftPower number used for the left side of the robot
     * @param rightPower number used for the right side of the robot
     */
    void setSidePowers(double leftPower, double rightPower);
}