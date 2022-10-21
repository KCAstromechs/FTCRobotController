package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="FC", group="Iterative Opmode")
public class M1FCTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public M1_Robot_Base rb;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    double leftPower;
    double rightPower;
    double trigger;
    BNO055IMU imu;





    @Override
    public void init() {

        rb = new M1_Robot_Base(hardwareMap, telemetry);
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        backRight = hardwareMap.get(DcMotor.class,"backRight");
        BNO055IMU.Parameters IMUParams = new BNO055IMU.Parameters();IMUParams.mode = BNO055IMU.SensorMode.IMU;
        IMUParams.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu.initialize(IMUParams);
        while(!imu.isGyroCalibrated());


        double waitTime = getRuntime()+ 1.;
        //while(getRuntime()<waitTime);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();





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
        double yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;  // 0-90-180-
        
        // negative value -> go left -> positive power to left wheels, negative power to right wheels
        // positive value -> go right -> positive power to right wheels, negative power to left wheels
        // without other movement ex: frontRight.setPower(turnPower);
        double turnPower = (gamepad1.left_stick_x);

        double robotX = (gamepad1.right_stick_x*Math.cos(yAngle))+(gamepad1.right_stick_y*Math.cos(yAngle+(PI/2)));
        double robotY = (gamepad1.right_stick_x*Math.sin(yAngle))+(gamepad1.right_stick_y*Math.sin(yAngle+(PI/2)));

        if((Math.abs(robotX+robotY))>1){
            frontLeft.setPower((robotX+robotY)/(robotX+robotY));
            frontRight.setPower((robotX-robotY) /(robotX+robotY));
            backRight.setPower((robotX+robotY) /(robotX+robotY));
            backLeft.setPower((robotX-robotY) /(robotX+robotY));
        }

        else if(Math.abs((robotX-robotY))>1){
            frontLeft.setPower((robotX+robotY)/(robotX-robotY));
            frontRight.setPower((robotX-robotY)/(robotX-robotY));
            backRight.setPower((robotX+robotY)/(robotX-robotY));
            backLeft.setPower((robotX-robotY)/(robotX-robotY));
        }

        else {
            frontLeft.setPower((robotX+robotY));
            frontRight.setPower((robotX-robotY));
            backRight.setPower((robotX+robotY));
            backLeft.setPower((robotX-robotY));

        }


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
