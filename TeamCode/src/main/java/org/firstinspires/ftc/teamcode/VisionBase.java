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

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

class VisionBase {

    //----------------------------------------------------------------------------------------------
    // State
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

    // analyzing related variables
    int green = -16776961;
    int RED = -65536;
    int WHITE = 0xffffffff;
    int minX = 190;
    int maxX = 600;
    int minY = 230;
    int maxY= 350;
    int analyzedWidth = (maxX-minX);
    int analyzedHeight = (maxY-minY);
    int analyzedPixels = (analyzedWidth * analyzedHeight); // if analyzing every pixel
    int dividerA = (minX + (analyzedWidth/3));
    int dividerB = (minX + (analyzedWidth/3*2));
    int minAvgGreen = 30; // lowball value, properly calculate this with testing
    String mostGreen;
    String preferredPosition = "RIGHT";

    //----------------------------------------------------------------------------------------------
    // Interacting with this Class
    //----------------------------------------------------------------------------------------------

    // do this at the beginning
    public void initVision() {
        callbackHandler = CallbackLooper.getDefault().getHandler();
        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        initializeFrameQueue(2);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
    }

    // call this to analyze and take the picture
    public void runVision() {
        // if something goes wrong and mostGreen is not updated, we will go to our preferred position
        mostGreen = preferredPosition;
        try {
            // if we can't access our camera, stop running the code
            openCamera();
            if (camera == null) {
                return;
            }
            startCamera();
            if (cameraCaptureSession == null) {
                return;
            }
            // grab our frame and then do something with it
            Bitmap bmp = frameQueue.poll();
            if (bmp != null) {
                onNewFrame(bmp);
            }
            // give info
            telemetry.update();

        } finally {
            closeCamera();
        }
    }

    // do stuff with the frame
    private void onNewFrame(Bitmap frame) {
        annotateBitmap(frame);
        analyzeBitmap(frame);
        saveBitmap(frame);
        frame.recycle(); // not strictly necessary, but helpful
    }

    //----------------------------------------------------------------------------------------------
    // Vision Analysis Operations
    //----------------------------------------------------------------------------------------------
    private void annotateBitmap(Bitmap bitmap){
        // plot x axis
        int imageWidth = bitmap.getWidth();
        for (int x = 0; x < imageWidth; x++) {
            bitmap.setPixel(x,0,green);
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

    private void analyzeBitmap(Bitmap bitmap){
        int color = 0;
        int greenValue = 0;
        int greenA = 0;
        int greenB = 0;
        int greenC = 0;
        int greenAvgA = 0;
        int greenAvgB = 0;
        int greenAvgC = 0;
        String marker = "DETECTED";
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
        // let's make sure we can see the team marker
        greenAvgA = (greenA / analyzedPixels);
        greenAvgB = (greenB / analyzedPixels);
        greenAvgC = (greenC / analyzedPixels);
        if (greenAvgA < minAvgGreen && greenAvgB < minAvgGreen && greenAvgC < minAvgGreen) {
            marker = "NOT DETECTED";
            mostGreen = preferredPosition;
        }

        // tell me which one has the most green unless we think we don't see the team marker
        if (marker != "NOT DETECTED") {
            if( greenA >= greenB && greenA >= greenC)
                mostGreen = "LEFT";
            else if (greenB >= greenA && greenB >= greenC)
                mostGreen = "CENTER";
            else
                mostGreen = "RIGHT";
        }

        // output
        telemetry.addData("MARKER", marker);
        telemetry.addData("LEFT", greenA);
        telemetry.addData("CENTER", greenB);
        telemetry.addData("RIGHT", greenC);
        telemetry.addData("Section", mostGreen);
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

}
