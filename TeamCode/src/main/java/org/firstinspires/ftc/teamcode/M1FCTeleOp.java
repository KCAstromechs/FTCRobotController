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
public class M1FCTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public M1_Robot_Base rb;
    double fieldAngle;
    BNO055IMU imu;
    double desiredAngle;
    boolean dpad = false;
    double errorK = 0.015;

    @Override
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rb = new M1_Robot_Base(hardwareMap, telemetry, true);

        // original angle
        fieldAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

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
        double zAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        if (dpad = true) {
            if (Math.abs(zAngle - desiredAngle) < errorK) {
                rb.dpadDirection(0);
                dpad = false;
                rb.dpadTurn(false);
            }
        }

        // dpad turning
        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            dpad = true;
            rb.dpadTurn(true);
            // what is our desired angle?
            if (gamepad1.dpad_up) {
                desiredAngle = 0;
            }
            else if (gamepad1.dpad_down) {
                desiredAngle = PI;
            }
            else if (gamepad1.dpad_left) {
                desiredAngle = -PI/2;
            }
            else if (gamepad1.dpad_right) {
                desiredAngle = PI/2;
            }
            else {
                desiredAngle = zAngle;
            }

            // are we at the angle?
            if (Math.abs(zAngle - desiredAngle) < errorK){
                rb.dpadDirection(0);
            }
            // which direction is fastest?
            else if (desiredAngle < zAngle) {
                rb.dpadDirection(1);
            } else if (desiredAngle > zAngle){
                rb.dpadDirection(-1);
            }
            // discontinuity - are we going the shortest path?
            if (Math.abs(zAngle-desiredAngle) > PI) {
                rb.dpadInverse();
            }
        }
        // if button toggle, switch speed
        if (gamepad1.left_bumper) {
            rb.FCSlowMode(true);
        } else if (gamepad1.right_bumper) {
            rb.FCSlowMode(false);
        }

        // FC Drive
        double turnPower = (gamepad1.left_stick_x);
        double inputX = gamepad1.right_stick_x;
        double inputY = -gamepad1.right_stick_y;

        rb.FCDrive(inputX,inputY,turnPower);

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
        
        rb.performFCUpdates();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop () {
    }

}

