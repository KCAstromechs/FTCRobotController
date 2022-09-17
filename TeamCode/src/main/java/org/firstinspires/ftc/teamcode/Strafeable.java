package org.firstinspires.ftc.teamcode;

/**
 * used for robots that have the capability to strafe, is used as an addition to a tank drive
 */
public interface Strafeable {

    /**
     * sets up the strafePower value to be used by performUpdates
     * @param strafePower equivalent to the "trigger" variable in the mecanum drive
     */
    void strafe(double strafePower);
}