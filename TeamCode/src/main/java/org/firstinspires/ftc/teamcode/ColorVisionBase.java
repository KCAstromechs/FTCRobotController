
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorVisionBase {

    private CameraBase cb;

    //----------------------------------------------------------------------------------------------
    // Variables
    //----------------------------------------------------------------------------------------------

    // color variables
    enum ZONE {
        ONE,
        TWO,
        THREE,
        NOT_DETECTED
    }
    ZONE mostRGB = ZONE.NOT_DETECTED;

    // robot variables
    HardwareMap hardwareMap;
    Telemetry telemetry;

    //----------------------------------------------------------------------------------------------
    // Utilization in Auto / Interact with this Class
    //----------------------------------------------------------------------------------------------

    public ColorVisionBase(HardwareMap _hardwareMap, Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;
        cb = new CameraBase(_hardwareMap, _telemetry);
    }

    // do this at the beginning
    public void initVision() {
        cb.initCamera();
    }

    public ZONE findZone(int minX, int maxX, int minY, int maxY, boolean save) {
        int color = 0;
        int redValue = 0;
        int greenValue = 0;
        int blueValue = 0;
        int pixelCountR = 0;
        int pixelCountG = 0;
        int pixelCountB = 0;
        int detectionThreshold = 50;
        int minColorDifference = 20;
        int countThreshold = 500;

        // retrieve bitmap
        Bitmap bitmap = cb.returnBitmap(minX, maxX, minY, maxY, save);
        if (bitmap == null) {
            return ZONE.NOT_DETECTED;
        }

        // color correction
        color = bitmap.getPixel(maxX,minY);
        redValue = Color.red(color);
        greenValue = Color.green(color);
        blueValue = Color.blue(color);
        double redToGreen = (double) redValue / (double) greenValue;
        double redToBlue = (double) redValue / (double) blueValue;
        telemetry.addData("redToGreen", redToGreen);
        telemetry.addData("redToBlue", redToBlue);
        // loop thru bitmap
        for (int x = minX; x < maxX; x++) {
            for (int y = minY; y < maxY; y++) {
                // get color for this coordinate
                color = bitmap.getPixel(x,y);
                redValue = Color.red(color);
                greenValue = (int) ((double)Color.green(color) * redToGreen);
                blueValue = (int) ((double)Color.blue(color) * redToBlue);
                // color
                if (redValue > detectionThreshold || greenValue > detectionThreshold || blueValue > detectionThreshold) {
                    if (redValue > greenValue + minColorDifference && redValue > blueValue + minColorDifference)
                        pixelCountR++;
                    else if (greenValue > redValue + minColorDifference && greenValue > blueValue)
                        pixelCountG++;
                    else if (blueValue > redValue + minColorDifference && blueValue > greenValue + minColorDifference)
                        pixelCountB++;
                }

            }
        }
        telemetry.addData("red pixels",pixelCountR);
        telemetry.addData("green pixels",pixelCountG);
        telemetry.addData("blue pixels", pixelCountB);
        if (pixelCountR > countThreshold || pixelCountG > countThreshold || pixelCountB > countThreshold) {
            if (pixelCountR < 1000 && pixelCountB < 1000){
                mostRGB = ZONE.TWO;
            }
            else if(pixelCountR > pixelCountG && pixelCountR > pixelCountB)
                mostRGB = ZONE.ONE;
            else if (pixelCountG > pixelCountR && pixelCountG > pixelCountB)
                mostRGB = ZONE.TWO;
            else if (pixelCountB > pixelCountR && pixelCountB > pixelCountG)
                mostRGB = ZONE.THREE;
            else
                mostRGB = ZONE.NOT_DETECTED;
        }
        bitmap.recycle();
        return mostRGB;
    }

}
