/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="M2 TeleOp", group="Iterative Opmode")
public class M2TeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor shooter = null;
    private DcMotor rotator = null;
    private DcMotor intake = null;
    private DcMotor encoderY = null;
    private DcMotor encoderX = null;
    private Servo wobbleGoal = null;
    private BNO055IMU imu;

    double leftPower;
    double rightPower;
    double trigger;
    double servoPosition = 0;
    double shooterPower = .62;
    boolean aWasPressed, bWasPressed;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        rotator = hardwareMap.get(DcMotor.class, "rotator");
        encoderY = hardwareMap.get(DcMotor.class, "encoderY");
        wobbleGoal = hardwareMap.get(Servo.class,"wobbleGoal");
        imu = hardwareMap.get(BNO055IMU.class,"imu");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        rotator.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        // A behavior for the motors that allows them to drift forward less
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // RESET ENCODERS
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // START THE ENCODERS
        encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //All the gyro setup
        float yAngle;
        BNO055IMU.Parameters IMUParams = new BNO055IMU.Parameters();
        IMUParams.mode = BNO055IMU.SensorMode.IMU;
        IMUParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(IMUParams);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /*
        float zAngle;
        float yAngle;
        float xAngle;
        xAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        BNO055IMU.Parameters IMUParams = new BNO055IMU.Parameters();IMUParams.mode = BNO055IMU.SensorMode.IMU;
        IMUParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        telemetry.addData( "y angle:", yAngle);
        telemetry.update();
         */
        //SET UP  VARIABLE FOR EACH WHEEL


        //turbo button
        if (gamepad1.left_bumper){
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
        }
        else if(gamepad1.right_bumper) {
            leftPower = -gamepad1.left_stick_y/4;
            rightPower = -gamepad1.right_stick_y/4;
        }
        else {
            leftPower = -gamepad1.left_stick_y/2;
            rightPower= -gamepad1.right_stick_y/2;
        }
        trigger = (gamepad1.right_trigger-gamepad1.left_trigger)/2;
        //mecanum, wheels diagonal from each other go the same direction
        // right front is always -, and so is its diagonal friend
        frontRight.setPower(rightPower-trigger);
        backRight.setPower(rightPower+trigger);
        frontLeft.setPower(leftPower+trigger);
        backLeft.setPower(leftPower-trigger);


    // shooting and intake
    //intake
     if(gamepad2.left_bumper){
         intake.setPower(1);
     }
     else if(gamepad2.dpad_down){
         intake.setPower(-1);
     }
     else{
         intake.setPower(0);
     }
     // rotator
     if (gamepad2.dpad_left){
         rotator.setPower(1);
     }
     else if(gamepad2.dpad_right){
         rotator.setPower(-1);
     }
     else{
         rotator.setPower(0);
     }

      // power testing
    if(gamepad2.a) {
        if (!aWasPressed) {
            aWasPressed = true;
            shooterPower -= .01;
        }
    }
    else{
        aWasPressed = false;
    }

    if(gamepad2.b) {
        if (!bWasPressed) {
            bWasPressed = true;
            shooterPower += .01;
        }
    }
    else{
        bWasPressed = false;
    }

     // shooting
    if(gamepad2.right_bumper) {
        shooter.setPower(shooterPower);
    }
    else if(gamepad2.dpad_up) {
        shooter.setPower(-shooterPower);
    }
    else {
        shooter.setPower(0);
    }




        //Wobble Goal Servo
        //Up Position
        if(gamepad2.y){
           // wobbleGoal.setPosition(0);
            if (servoPosition<.6) servoPosition+= .005;
            wobbleGoal.setPosition(servoPosition);

        }
        /*
        //Down Position
        if (gamepad2.x){
            //wobbleGoal.setPosition(.0);
            if (servoPosition>.5) servoPosition-= .005;
            wobbleGoal.setPosition(servoPosition);
        }
        //Reset Position
        if(gamepad2.a){
            wobbleGoal.setPosition(1);
        }
        //encoder reset
        if(gamepad1.b){
            encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


         */


        telemetry.addData("encoderY", encoderY.getCurrentPosition());
        telemetry.addData("shooterPower", shooterPower);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}