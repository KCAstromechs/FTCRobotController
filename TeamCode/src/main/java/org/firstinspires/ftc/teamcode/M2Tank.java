package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="Tank Test", group="Iterative Opmode")
public class M2Tank extends OpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public M2RobotBase rb;


    @Override
    public void init() {

        rb = new M2RobotBase(hardwareMap, telemetry, false);


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

        rb.setSidePowers(-gamepad1.left_stick_y/2,-gamepad1.right_stick_y/2);
        rb.strafe((gamepad1.right_trigger-gamepad1.left_trigger)/2);


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

        rb.performUpdates();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop () {
    }

}