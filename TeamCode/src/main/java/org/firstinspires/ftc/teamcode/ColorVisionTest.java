


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RGB Vision Test")
public class ColorVisionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ColorVisionBase vision = new ColorVisionBase(hardwareMap, telemetry);

        // do this before match start
        vision.initVision();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // now let's run vision, full image is 640 x 480
        ColorVisionBase.ZONE zone = vision.findZone(215,280,250,370, true);
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
