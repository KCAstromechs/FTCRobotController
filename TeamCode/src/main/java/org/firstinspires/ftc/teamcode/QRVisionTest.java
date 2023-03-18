


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="QR Vision Test")
public class QRVisionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        QRVisionBase vision = new QRVisionBase(hardwareMap, telemetry);

        // do this before match start
        vision.initVision();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // now let's run vision, full image is 640 x 480
        QRVisionBase.ZONE zone = QRVisionBase.ZONE.NOT_DETECTED;
        zone = vision.findZone(0,639,0,479, true);
        if (zone == QRVisionBase.ZONE.ONE) {
            telemetry.addData("Final Answer", "ONE");
        }
        else if (zone == QRVisionBase.ZONE.TWO) {
            telemetry.addData("Final Answer", "TWO");
        }
        else if (zone == QRVisionBase.ZONE.THREE) {
            telemetry.addData("Final Answer", "THREE");
        }
        else {
            telemetry.addData("Final Answer", "NOT DETECTED");
        }
        telemetry.update();
        sleep(5000);

    }
}
