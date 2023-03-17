
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class QRVisionBase {

    private CameraBase cb;

    // robot variables
    HardwareMap hardwareMap;
    Telemetry telemetry;

    //----------------------------------------------------------------------------------------------
    // Utilization in Auto / Interact with this Class
    //----------------------------------------------------------------------------------------------

    public QRVisionBase(HardwareMap _hardwareMap, Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;
    }

    // do this at the beginning
    public void initVision() {
        cb.initCamera();
    }

    public int findZone(int minX, int maxX, int minY, int maxY, boolean save) {
        int color = 0;
        int redValue = 0;
        int greenValue = 0;
        int blueValue = 0;
        int pixelCountR = 0;
        int pixelCountG = 0;
        int pixelCountB = 0;
        int detectionThreshold = 50;
        int minColorDifference = 20;

        // retrieve bitmap
        Bitmap bitmap = cb.returnBitmap(minX, maxX, minY, maxY, save);
        if (bitmap == null) {
            return 0;
        }
        return 1;
    }
}
