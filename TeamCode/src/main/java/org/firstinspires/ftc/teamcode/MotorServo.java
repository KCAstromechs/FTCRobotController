package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorServo {
    DcMotor thisMotor;

    public MotorServo(DcMotor motor) {
        thisMotor = motor;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // motor.setMode(DcMotor.RunMode.R);



    }
}
