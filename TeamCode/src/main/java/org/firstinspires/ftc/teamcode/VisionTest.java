
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Vision Test")
public class VisionTest extends LinearOpMode {

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
        VisionBase.COLOR color = vision.findTSEPosition(20,620,20,460, true);

        if (color == VisionBase.COLOR.RED) {
            telemetry.addData("Final Answer", "RED");
        }
        else if (color == VisionBase.COLOR.GREEN) {
            telemetry.addData("Final Answer", "GREEN");
        }
        else if (color == VisionBase.COLOR.BLUE) {
            telemetry.addData("Final Answer", "BLUE");
        }
        else {
            telemetry.addData("Final Answer", "NOT DETECTED");
        }
        telemetry.update();
        sleep(10000);

    }
}