
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Point;
import android.graphics.Rect;
import android.os.Bundle;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import com.google.android.gms.tasks.OnFailureListener;
import com.google.android.gms.tasks.OnSuccessListener;
import com.google.android.gms.tasks.Task;
import com.google.mlkit.vision.barcode.common.Barcode;
import com.google.mlkit.vision.barcode.BarcodeScanner;
import com.google.mlkit.vision.barcode.BarcodeScannerOptions;
import com.google.mlkit.vision.barcode.BarcodeScanning;
import com.google.mlkit.vision.common.InputImage;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class QRVisionBase {

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

    // robot variables
    HardwareMap hardwareMap;
    Telemetry telemetry;

    //----------------------------------------------------------------------------------------------
    // Utilization in Auto / Interact with this Class
    //----------------------------------------------------------------------------------------------

    public QRVisionBase(HardwareMap _hardwareMap, Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;
        cb = new CameraBase(_hardwareMap, _telemetry);
    }

    // do this at the beginning
    public void initVision() {
        cb.initCamera();
    }

    public ZONE findZone(int minX, int maxX, int minY, int maxY, boolean save) {
        // set-up barcode scanning
        BarcodeScannerOptions options =
                new BarcodeScannerOptions.Builder()
                        .setBarcodeFormats(
                                Barcode.FORMAT_QR_CODE)
                        .build();
        BarcodeScanner scanner = BarcodeScanning.getClient();
        // retrieve bitmap
        Bitmap bitmap = cb.returnBitmap(minX, maxX, minY, maxY, save);
        if (bitmap == null) {
            return ZONE.NOT_DETECTED;
        }
        // scan barcode
        InputImage image = InputImage.fromBitmap(bitmap, 0);
        Task<List<Barcode>> result = scanner.process(image)
                .addOnSuccessListener(new OnSuccessListener<List<Barcode>>() {
                    @Override
                    public void onSuccess(List<Barcode> barcodes) {
                        // Task completed successfully
                        for (Barcode barcode: barcodes) {
                            Rect bounds = barcode.getBoundingBox();
                            Point[] corners = barcode.getCornerPoints();

                            String rawValue = barcode.getRawValue();

                            int valueType = barcode.getValueType();
                            }
                        }
                })
                .addOnFailureListener(new OnFailureListener() {
                    @Override
                    public void onFailure(@NonNull Exception e) {
                        // Task failed with an exception
                        // ...
                    }
                });

        return ZONE.ONE;
    }

}
