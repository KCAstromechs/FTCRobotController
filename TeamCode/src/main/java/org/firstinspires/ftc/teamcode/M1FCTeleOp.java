package org.firstinspires.ftc.teamcode;

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
    double K = 1.5;




    @Override
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        rb = new M1_Robot_Base(hardwareMap, telemetry);
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


        // PART 1 - THE MATH

        double  yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
        // STEP 1
        double thetaAngle = Math.atan2(-gamepad1.left_stick_y,gamepad1.left_stick_x);
        telemetry.addData("thetaAngle:", thetaAngle);
        // STEP 2
        double gammaAngle = thetaAngle - yAngle;
        telemetry.addData("gammaAngle:", gammaAngle);
        // STEP 3
        double hyp = Math.sqrt(gamepad1.left_stick_y * gamepad1.left_stick_y + gamepad1.left_stick_x *gamepad1.left_stick_x);
        telemetry.addData("hyp:", hyp);
        // STEP 4
        double robotY = hyp * Math.sin(gammaAngle);
        double robotX = hyp * Math.cos(gammaAngle);

        // PART 2 - THE APPLICATION

        frontRight.setPower(robotY - robotX - (gamepad1.right_stick_x) - gamepad2.right_stick_x);
        backRight.setPower(robotY + robotX - (gamepad1.right_stick_x) - gamepad2.right_stick_x);
        frontLeft.setPower(robotY + robotX + (gamepad1.right_stick_x) + gamepad2.right_stick_x);
        backLeft.setPower(robotY - robotX + (gamepad1.right_stick_x) + gamepad2.right_stick_x);



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
