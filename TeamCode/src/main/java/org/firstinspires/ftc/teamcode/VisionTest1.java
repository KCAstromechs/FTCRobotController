// Specifically for the left blue starting position.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Vision Test1 (Java)")
public class VisionTest1 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        VisionBase vision = new VisionBase(hardwareMap, telemetry);


        // do this before match start
        vision.initVision();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // now let's run vision, full image is 640 x 480
        // values for left line up:
        VisionBase.COLOR SpikeMarkLeft = vision.findRGB(0, 52, 211, 290, true);
        VisionBase.COLOR SpikeMarkCenter = vision.findRGB(272, 344, 205, 275, true);
        VisionBase.COLOR SpikeMarkRight = vision.findRGB(572, 638, 230, 325, true);
        if (SpikeMarkLeft == VisionBase.COLOR.RED) {
            telemetry.addData("Final Answer", "Left RED");
        }
        else if (SpikeMarkCenter == VisionBase.COLOR.RED) {
            telemetry.addData("Final Answer", "Center RED");
        }
        else if (SpikeMarkRight == VisionBase.COLOR.RED) {
            telemetry.addData("Final Answer", "Right RED");
        }
        else {
            telemetry.addData("Final Answer", "RED NOT DETECTED");
        }

        telemetry.update();

        sleep(5000);

    }
}