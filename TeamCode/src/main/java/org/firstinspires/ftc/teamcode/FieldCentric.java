package org.firstinspires.ftc.teamcode;

/**
 * used for robots that utilize a field centric drive.
 */
public interface FieldCentric {

    void FCDrive(double inputX, double inputY, double turnPower);

    void FCSlowMode(boolean slow);

    void dpadTurn (boolean dpad);

    void dpadDirection (double direction);

    void dpadInverse ();
}