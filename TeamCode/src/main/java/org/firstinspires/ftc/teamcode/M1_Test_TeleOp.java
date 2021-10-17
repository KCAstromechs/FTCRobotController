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
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name=" M1 Test", group="Iterative Opmode")
public class M1_Test_TeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor carouselMover = null;
    private BNO055IMU imu;

    double leftPower;
    double rightPower;
    double trigger;
    double carouselPower;


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
        carouselMover = hardwareMap.get(DcMotor.class, "carouselMover");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // A behavior for the motors that allows them to drift forward less
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //All the gyro setup
        float yAngle;
       /* BNO055IMU.Parameters IMUParams = new BNO055IMU.Parameters();
        IMUParams.mode = BNO055IMU.SensorMode.IMU;
        IMUParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(IMUParams);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        */
       carouselPower=0;
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


        //slow button
        if (gamepad1.left_bumper){
            leftPower = -gamepad1.left_stick_y/4;
            rightPower = -gamepad1.right_stick_y/4;
        }
        else{
            //hey, it's negative because up on the joystick is negative, and we need to make sure that the number it returns is positive
            leftPower = -gamepad1.left_stick_y/2;
            rightPower = -gamepad1.right_stick_y/2;
        }

        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);



        //-.65 and .65
        if(gamepad1.dpad_up){
            carouselMover.setPower(carouselPower =+.05);
        }
        if (gamepad1.dpad_down){
            carouselMover.setPower(carouselPower -=.05);
        }
        if(gamepad1.a){
            carouselMover.setPower(-carouselPower);
        }
        if (gamepad1.b){
            carouselMover.setPower(0);
        }


        trigger = (gamepad1.right_trigger - gamepad1.left_trigger);

        telemetry.addData("carousel power", carouselPower);
        telemetry.addData("frontRight", frontRight.getPower());
        telemetry.addData("frontLeft", frontRight.getPower());
        telemetry.addData("backRight", frontRight.getPower());
        telemetry.addData("backLeft", frontRight.getPower());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
