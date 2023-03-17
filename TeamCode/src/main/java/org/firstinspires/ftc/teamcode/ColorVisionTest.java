


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Vision Test")
public class ColorVisionTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        ColorVisionBase vision = new ColorVisionBase(hardwareMap, telemetry);

        // do this before match start
        vision.initVision();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // now let's run vision, full image is 640 x 480
        // values for left line up:
        ColorVisionBase.ZONE zone = vision.findZone(187,268,200,280, true);
        if (zone == ColorVisionBase.ZONE.ONE) {
            telemetry.addData("Final Answer", "ONE");
        }
        else if (zone == ColorVisionBase.ZONE.TWO) {
            telemetry.addData("Final Answer", "TWO");
        }
        else if (zone == ColorVisionBase.ZONE.THREE) {
            telemetry.addData("Final Answer", "THREE");
        }
        else {
            telemetry.addData("Final Answer", "NOT DETECTED");
        }
        telemetry.update();
        sleep(5000);

    }
}
