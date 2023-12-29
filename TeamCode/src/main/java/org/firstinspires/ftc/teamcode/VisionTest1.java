// Created by Astromechs from the 2022-2023 FTC season (POWERPLAY)

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
        VisionBase.COLOR SpikeMarkRight = vision.findRGB(572, 640, 210, 293, true);
        if (SpikeMarkCenter == VisionBase.COLOR.RED) {
            telemetry.addData("Final Answer", "RED");
        }
        else if (SpikeMarkCenter == VisionBase.COLOR.GREEN) {
            telemetry.addData("Final Answer", "GREEN");
        }
        else if (SpikeMarkCenter == VisionBase.COLOR.BLUE) {
            telemetry.addData("Final Answer", "BLUE");
        }
        else {
            telemetry.addData("Final Answer", "NOT DETECTED");
        }
        telemetry.update();
        sleep(5000);

    }
}