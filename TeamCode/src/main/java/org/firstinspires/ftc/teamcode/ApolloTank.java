package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TANK", group="Iterative Opmode")
public class ApolloTank extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public ApolloBase rb;
    double leftPower;
    double rightPower;
    double trigger;
    double K = 1.5;


    @Override
    public void init() {

        rb = new ApolloBase(hardwareMap, telemetry);


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


        // if button toggle, switch speed
        if (gamepad1.left_bumper){
            K = 3;
        }
        else if (gamepad1.right_bumper) {
            K = 1;
        }

        leftPower = -gamepad1.left_stick_y/K;
        rightPower = -gamepad1.right_stick_y/K;

        // logical tank, mechanical strafe
        rb.setSidePowers(leftPower, rightPower);

        // logical strafe, mechanical tank
        trigger=(gamepad1.right_trigger-gamepad1.left_trigger);
        rb.strafe(trigger/K);

        rb.performUpdates();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

