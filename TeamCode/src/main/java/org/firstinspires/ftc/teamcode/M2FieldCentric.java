package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="Field Centric Beetle", group="Iterative Opmode")
public class M2FieldCentric extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public M2RobotBase rb;
    double fieldAngle;
    BNO055IMU imu;
    double angleOffset = 0.0;

    @Override
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rb = new M2RobotBase(hardwareMap, telemetry, true);

        // original angle
        fieldAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Angle", fieldAngle);
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
        // input from gamepad sticks
        double turnPower = (gamepad1.left_stick_x);
        double inputX = gamepad1.right_stick_x;
        double inputY = -gamepad1.right_stick_y;
        if(Math.abs(inputX)<0.2) inputX = 0;
        if(Math.abs(inputY)<0.2) inputY = 0;

        // if dpad button, change input values
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            if (gamepad1.dpad_up){
                inputX = 0;
                inputY = 0.5;
            }
            else if (gamepad1.dpad_down){
                inputX = 0;
                inputY = -0.5;
            }
            else if (gamepad1.dpad_left){
                inputX = -0.5;
                inputY = 0;
            }
            else if (gamepad1.dpad_right){
                inputX = 0.5;
                inputY = 0;
            }
        }

        // drive now :)
        rb.FCDrive(Math.cbrt(inputX)/2, Math.cbrt(inputY)/2, Math.cbrt(turnPower)/2);


        // RESET ANGLE ?
        if (gamepad1.left_bumper) {
            angleOffset = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        }

        //COLLECTOR
        if (gamepad2.a){
            rb.collectorOpen();
        }

        if (gamepad2.b){
            rb.collectorClose();
        }

        if(gamepad2.x){
            try {
                rb.lifterResetDown();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if(gamepad2.y){
            try {
                rb.lifterResetUp();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        //LIFTER
        rb.lifterControl((int)(-gamepad2.right_stick_y*100.0));

        if(gamepad2.dpad_up){
            rb.lifterHigh();
        }
        if(gamepad2.dpad_right){
            rb.lifterMedium();
        }
        if(gamepad2.dpad_down){
            rb.lifterLow();
        }
        if(gamepad2.dpad_left){
            rb.lifterZero();
        }

        rb.performFCUpdates(angleOffset);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop () {
    }

}