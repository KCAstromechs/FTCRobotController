
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import androidx.annotation.NonNull;

import com.google.android.gms.tasks.OnCompleteListener;
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

import static java.lang.Thread.sleep;

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

    ZONE currentZone = ZONE.THREE;
    boolean barcodeComplete = false;
    String rawValue = "nothing!";

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
            telemetry.addData("bitmap","NO BITMAP??");
            telemetry.update();
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
                            rawValue = barcode.getRawValue();
                            //currentZone = ZONE.ONE;
                            //telemetry.addData("rawValue", rawValue);

                        }
                        barcodeComplete = true;
                    }
                })
                .addOnFailureListener(new OnFailureListener() {
                    @Override
                    public void onFailure(@NonNull Exception e) {
                        // Task failed with an exception
                        // ...
                        //telemetry.addData("success?","NO");
                        //currentZone = ZONE.TWO;
                        barcodeComplete = true;
                        e.printStackTrace();
                    }
                })
                .addOnCompleteListener(new OnCompleteListener<List<Barcode>>() {
                    @Override
                    public void onComplete(@NonNull Task<List<Barcode>> task) {
                        barcodeComplete = true;
                    }
                });

        while(!barcodeComplete) {
            try {
                sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        bitmap.recycle();
        if (rawValue == "3409-1"){
            currentZone = ZONE.ONE;
        }
        else if (rawValue == "3409-2"){
            currentZone = ZONE.TWO;
        }
        else if (rawValue == "3409-3"){
            currentZone = ZONE.THREE;
        }
        else {
            currentZone = ZONE.NOT_DETECTED;
        }
        telemetry.addData("rawValue", rawValue);
        return currentZone; // placeholder
    }

}
