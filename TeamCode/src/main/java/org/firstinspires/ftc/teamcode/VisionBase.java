/* Copyright (c) 2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.os.Handler;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

import androidx.annotation.NonNull;


public class VisionBase {

    enum TSEPosition {
        LEFT,
        CENTER,
        RIGHT,
        NOT_DETECTED
    }

    //----------------------------------------------------------------------------------------------
    // Analyzing-Related Variables
    //----------------------------------------------------------------------------------------------

    // color constants
    int BLUE = -16776961;
    int RED = -65536;
    int WHITE = 0xffffffff;
    // mapping values
    int minX = 0;
    int maxX = 0;
    int minY = 0;
    int maxY = 0;
    int analyzedWidth = 0;
    int analyzedHeight = 0;
    int analyzedPixels = 0;
    int dividerA = 0;
    int dividerB = 0;
    // other
    TSEPosition mostGreen = TSEPosition.NOT_DETECTED;
    TSEPosition leastBlue = TSEPosition.NOT_DETECTED;
    TSEPosition mostGreenBlueDifference = TSEPosition.NOT_DETECTED;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    //----------------------------------------------------------------------------------------------
    // Stolen Camera Variables
    //----------------------------------------------------------------------------------------------

    private static final String TAG = "Webcam Sample";

    /** How long we are to wait to be granted permission to use the camera before giving up. Here,
     * we wait indefinitely */
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;

    /** State regarding our interaction with the camera */
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;

    /** The queue into which all frames from the camera are placed as they become available.
     * Frames which are not processed by the OpMode are automatically discarded. */
    private EvictingBlockingQueue<Bitmap> frameQueue;

    /** State regarding where and how to save frames when the 'A' button is pressed. */
    private int captureCounter = 0;
    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    /** A utility object that indicates where the asynchronous callbacks from the camera
     * infrastructure are to run. In this OpMode, that's all hidden from you (but see {@link #startCamera}
     * if you're curious): no knowledge of multi-threading is needed here. */
    private Handler callbackHandler;

    //----------------------------------------------------------------------------------------------
    // Utilization in Auto / Interact with this Class
    //----------------------------------------------------------------------------------------------

    public VisionBase(HardwareMap _hardwareMap, Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;
    }

    // do this at the beginning
    public void initVision() {
        callbackHandler = CallbackLooper.getDefault().getHandler();
        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        initializeFrameQueue(2);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        telemetry.addData("VISION", "initialized");
        telemetry.update();
    }

    /**
     * findTSEPosition has three main parts: setting up the area to be analyzed, grabbing a frame from the camera, and returning either LEFT, RIGHT, CENTER, or NOT_DETECTED
     * The coordinate inputs are used to draw a box around the desired area. This box is broken into thirds defining LEFT, RIGHT, and CENTER.
     * Once a frame is taken from the camera, onNewFrame is called. It will analyze for mostGreen, leastBlue, or the highest GreenBlueDifference. Currently it is set to find the highest GreenBlueDifference.
     * After receiving an answer from one of these analyzing methods, the camera is closed and the position is returned.
     * @param _minX Starting x coordinate to be analyzed
     * @param _maxX Ending x coordinate to be analyzed
     * @param _minY Starting y coordinate to be analyzed
     * @param _maxY Ending y coordinate to be analyzed
     * @param save Do you want the image to be saved after analyzing?
     * @return TSEPosition : either LEFT, CENTER, RIGHT, or NOT_DETECTED
     */
    public TSEPosition findTSEPosition(int _minX, int _maxX, int _minY, int _maxY, boolean save) {
        // setting up coords input as global
        minX = _minX;
        maxX = _maxX;
        minY = _minY;
        maxY = _maxY;
        analyzedWidth = (maxX-minX);
        analyzedHeight = (maxY-minY);
        analyzedPixels = (analyzedWidth * analyzedHeight); // if analyzing every pixel
        dividerA = (minX + (analyzedWidth/3));
        dividerB = (minX + (analyzedWidth/3*2));

        // if something goes wrong with vision process, not detected will be returned
        TSEPosition ret = TSEPosition.NOT_DETECTED;
        try {
            openCamera();
            if (camera == null) {
                return TSEPosition.NOT_DETECTED;
            }
            startCamera();
            if (cameraCaptureSession == null) {
                return TSEPosition.NOT_DETECTED;
            }

            // loop until we receive an image from the camera
            boolean haveBitmap = false;
            while (!haveBitmap) {
                Bitmap bmp = frameQueue.poll();
                if (bmp != null) {
                    // if we have a frame, run operations and break from the loop
                    ret = onNewFrame(bmp, save);
                    haveBitmap = true;
                }
            }

        } finally {
            closeCamera();
        }
        return ret;
    }

    // do stuff with the frame
    private TSEPosition onNewFrame(Bitmap frame, boolean save) {
        TSEPosition ret = analyzeBitmapForGreenBlueDifference(frame);
        if (save == true) {
            annotateBitmap(frame);
            saveBitmap(frame);
        }
        frame.recycle(); // not strictly necessary, but helpful
        return ret;
    }

    //----------------------------------------------------------------------------------------------
    // Vision Analysis Operations
    //----------------------------------------------------------------------------------------------

    private void annotateBitmap(Bitmap bitmap){
        // plot x axis
        int imageWidth = bitmap.getWidth();
        for (int x = 0; x < imageWidth; x++) {
            bitmap.setPixel(x,0,BLUE);
        }
        // plot y axis
        int imageHeight = bitmap.getHeight();
        for (int y = 0; y < imageHeight; y++) {
            bitmap.setPixel(0,y,RED);
        }
        // draw out area to analyze
        // left vertical line
        for (int y=minY; y < maxY; y++ ){
            bitmap.setPixel(minX,y,WHITE);
        }
        // top horizontal line
        for (int x=minX; x < maxX; x++){
            bitmap.setPixel(x,maxY,WHITE);
        }
        // right vertical line
        for (int y=maxY; y > minY; y--){
            bitmap.setPixel(maxX,y,WHITE);
        }
        // bottom horizontal line
        for (int x=maxX; x > minX; x--){
            bitmap.setPixel(x,minY,WHITE);
        }

        // mark dividers
        for (int y=maxY; y > minY; y--){
            bitmap.setPixel(dividerA,y,WHITE);
        }
        for (int y=maxY; y > minY; y--){
            bitmap.setPixel(dividerB,y,WHITE);
        }
    }

    private TSEPosition analyzeBitmapForGreen(Bitmap bitmap){
        int color = 0;
        int greenValue = 0;
        int greenA = 0;
        int greenB = 0;
        int greenC = 0;
        float greenAvgA = 0;
        float greenAvgB = 0;
        float greenAvgC = 0;
        // loop thru image
        for (int x = minX; x < maxX; x++) {
            for (int y = minY; y < maxY; y++) {
                // get color for this coordinate
                color = bitmap.getPixel(x,y);
                greenValue = Color.green(color);
                // sort green value by which third of the bitmap it is located in
                if (x < dividerA){
                    greenA += greenValue;
                }
                else if (x > dividerA && x < dividerB){
                    greenB += greenValue;
                }
                else {
                    greenC += greenValue;
                }
            }
        }
        greenAvgA = ((float)greenA / (float)analyzedPixels);
        greenAvgB = ((float)greenB / (float)analyzedPixels);
        greenAvgC = ((float)greenC / (float)analyzedPixels);

        // tell me which one has the most green
        if( greenAvgA >= greenAvgB && greenAvgA >= greenAvgC)
            mostGreen = TSEPosition.LEFT;
        else if (greenAvgB >= greenAvgA && greenAvgB >= greenAvgC)
            mostGreen = TSEPosition.CENTER;
        else
            mostGreen = TSEPosition.RIGHT;

        // output
        /* telemetry.addData("LEFT", greenAvgA);
        telemetry.addData("CENTER", greenAvgB);
        telemetry.addData("RIGHT", greenAvgC);
        telemetry.addData("Section", mostGreen); */

        return mostGreen;
    }

    private TSEPosition analyzeBitmapForBlue(Bitmap bitmap){
        int color = 0;
        int blueValue = 0;
        int blueA = 0;
        int blueB = 0;
        int blueC = 0;
        float blueAvgA = 0;
        float blueAvgB = 0;
        float blueAvgC = 0;
        // loop thru image
        for (int x = minX; x < maxX; x++) {
            for (int y = minY; y < maxY; y++) {
                // get color for this coordinate
                color = bitmap.getPixel(x,y);
                blueValue = Color.blue(color);
                // sort green value by which third of the bitmap it is located in
                if (x < dividerA){
                    blueA += blueValue;
                }
                else if (x > dividerA && x < dividerB){
                    blueB += blueValue;
                }
                else {
                    blueC += blueValue;
                }
            }
        }
        blueAvgA = ((float)blueA / (float)analyzedPixels);
        blueAvgB = ((float)blueB / (float)analyzedPixels);
        blueAvgC = ((float)blueC / (float)analyzedPixels);

        // tell me which one has the LEAST BlUE
        if( blueAvgA <= blueAvgB && blueAvgA <= blueAvgC)
            leastBlue = TSEPosition.LEFT;
        else if (blueAvgB <= blueAvgA && blueAvgB <= blueAvgC)
            leastBlue = TSEPosition.CENTER;
        else
            leastBlue = TSEPosition.RIGHT;

        // output
        /* telemetry.addData("LEFT", blueAvgA);
        telemetry.addData("CENTER", blueAvgB);
        telemetry.addData("RIGHT", blueAvgC);
        telemetry.addData("Section", leastBlue); */

        return leastBlue;
    }

    private TSEPosition analyzeBitmapForGreenBlueDifference(Bitmap bitmap){
        int color = 0;
        int blueValue = 0;
        int greenValue = 0;
        int minColorDifference = 50; // difference between blue and green to be counted in the pixel count
        int pixelCountA = 0;
        int pixelCountB = 0;
        int pixelCountC = 0;
        int detectionThreshold = 50; // number of pixels counted to not count as "NOT DETECTED"

        // loop thru image
        for (int x = minX; x < maxX; x++) {
            for (int y = minY; y < maxY; y++) {
                // get color for this coordinate
                color = bitmap.getPixel(x,y);
                blueValue = Color.blue(color);
                greenValue = Color.green(color);
                if ((greenValue - blueValue) >= minColorDifference) {
                    if (x < dividerA){
                        pixelCountA += 1;
                    }
                    else if (x > dividerA && x < dividerB){
                        pixelCountB += 1;
                    }
                    else {
                        pixelCountC += 1;
                    }
                }
            }
        }
        // tell me which one has the biggest amount of pixels that have specified blue green difference unless under threshold
        if (pixelCountA > detectionThreshold || pixelCountB > detectionThreshold || pixelCountC > detectionThreshold) {
            if(pixelCountA > pixelCountB && pixelCountA > pixelCountC)
                mostGreenBlueDifference = TSEPosition.LEFT;
            else if (pixelCountB > pixelCountA && pixelCountB > pixelCountC)
                mostGreenBlueDifference = TSEPosition.CENTER;
            else if (pixelCountC > pixelCountA && pixelCountC > pixelCountB)
                mostGreenBlueDifference = TSEPosition.RIGHT;
            else
                mostGreenBlueDifference = TSEPosition.NOT_DETECTED;
        }

        // output
        /* telemetry.addData("LEFT", pixelCountA);
        telemetry.addData("CENTER", pixelCountB);
        telemetry.addData("RIGHT", pixelCountC);
        telemetry.addData("Section", mostGreenBlueDifference); */

        return mostGreenBlueDifference;
    }

    private void saveBitmap(Bitmap bitmap) {
        DateFormat dateFormat = new SimpleDateFormat("MM-dd__hh-mm-ss");
        String strDate = dateFormat.format(new Date());
        File file = new File(captureDirectory, String.format(Locale.getDefault(), strDate + ".png", captureCounter++));
        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                telemetry.log().add("captured %s", file.getName());
            }
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "exception in saveBitmap()");
            error("exception saving %s", file.getName());
        }
    }

    //----------------------------------------------------------------------------------------------
    // Stolen Camera Operations
    //----------------------------------------------------------------------------------------------

    private void initializeFrameQueue(int capacity) {
        /** The frame queue will automatically throw away bitmap frames if they are not processed
         * quickly by the OpMode. This avoids a buildup of frames in memory */
        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override public void accept(Bitmap frame) {
                // RobotLog.ii(TAG, "frame recycled w/o processing");
                frame.recycle(); // not strictly necessary, but helpful
            }
        });
    }

    private void openCamera() {
        if (camera != null) return; // be idempotent

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null) {
            error("camera not found or permission to use not granted: %s", cameraName);
        }
    }

    private void startCamera() {
        if (cameraCaptureSession != null) return; // be idempotent

        /** YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition
         * for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only*
         * image format supported by a camera */
        final int imageFormat = ImageFormat.YUY2;

        /** Verify that the image is supported, and fetch size and desired frame rate if so */
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            error("image format not supported");
            return;
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        /** Some of the logic below runs asynchronously on other threads. Use of the synchronizer
         * here allows us to wait in this method until all that asynchrony completes before returning. */
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            /** Create a session in which requests to capture frames can be made */
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        /** The session is ready to go. Start requesting frames */
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
                                        /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                         * for the duration of the callback. So we copy here manually. */
                                        Bitmap bmp = captureRequest.createEmptyBitmap();
                                        cameraFrame.copyToBitmap(bmp);
                                        frameQueue.offer(bmp);
                                    }
                                },
                                Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                    @Override public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                        RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                    }
                                })
                        );
                        synchronizer.finish(session);
                    } catch (CameraException|RuntimeException e) {
                        RobotLog.ee(TAG, e, "exception starting capture");
                        error("exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
            RobotLog.ee(TAG, e, "exception starting camera");
            error("exception starting camera");
            synchronizer.finish(null);
        }

        /** Wait for all the asynchrony to complete */
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        /** Retrieve the created session. This will be null on error. */
        cameraCaptureSession = synchronizer.getValue();
    }

    private void stopCamera() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    private void closeCamera() {
        stopCamera();
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Utilities
    //----------------------------------------------------------------------------------------------

    private void error(String msg) {
        telemetry.log().add(msg);
        telemetry.update();
    }
    private void error(String format, Object...args) {
        telemetry.log().add(format, args);
        telemetry.update();
    }

    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }

}