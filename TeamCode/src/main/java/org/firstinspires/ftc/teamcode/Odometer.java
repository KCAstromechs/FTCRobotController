package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Oodome", group="Iterative Opmode")
public class Odometer extends OpMode{
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

       rb.encoderTest();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop () {
    }

}