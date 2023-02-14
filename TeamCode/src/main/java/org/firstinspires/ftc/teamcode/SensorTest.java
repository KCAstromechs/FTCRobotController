package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Sensor Test", group="Iterative Opmode")
public class SensorTest extends OpMode{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public ColorSensor color;
    public DistanceSensor distance;




    @Override
    public void init() {

        color = hardwareMap.get(ColorSensor.class,"color");
        distance = hardwareMap.get(DistanceSensor.class,"distance");

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
    telemetry.addData("red sensed", color.red());
    telemetry.addData("green sensed", color.green());
    telemetry.addData("blue sensed", color.blue());
    telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
    telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop () {
    }

}