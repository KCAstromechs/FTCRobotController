
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;
import android.widget.ZoomControls;

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
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
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

public class CameraBase {


    //----------------------------------------------------------------------------------------------
    // Variables
    //----------------------------------------------------------------------------------------------

    // camera variables
    private static final String TAG = "Webcam Sample";

    /** How long we are to wait to be granted permission to use the camera before giving up. Here,
     * we wait indefinitely */
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;

    /** State regarding our interaction with the camera */
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;
    ExposureControl myExposureControl;
    GainControl myGainControl;

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
    // other
    HardwareMap hardwareMap;
    Telemetry telemetry;
    int EXPOSURE = 0;

    //----------------------------------------------------------------------------------------------
    // Interact with this Class
    //----------------------------------------------------------------------------------------------

    public CameraBase(HardwareMap _hardwareMap, Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;
    }

    // do this at the beginning
    public void initCamera() {
        callbackHandler = CallbackLooper.getDefault().getHandler();
        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        initializeFrameQueue(20);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        openCamera();
        startCamera();

        CameraControl c = camera.getControl(CameraControl.class);

        myExposureControl= camera.getControl(ExposureControl.class);
        myGainControl = camera.getControl(GainControl.class);

        myExposureControl.setMode(ExposureControl.Mode.Manual);
        myExposureControl.setExposure(EXPOSURE,TimeUnit.MILLISECONDS);
        myGainControl.setGain(20);

        long currentExposure = myExposureControl.getExposure(TimeUnit.MILLISECONDS);
        int currentGain = myGainControl.getGain();

        telemetry.addData("Exposure", currentExposure);
        telemetry.addData("Gain", currentGain);

        telemetry.addData("Camera", "initialized");
        telemetry.update();
    }

    public Bitmap returnBitmap(int _minX, int _maxX, int _minY, int _maxY, boolean save) {
        // setting up coords input as global
        minX = _minX;
        maxX = _maxX;
        minY = _minY;
        maxY = _maxY;
        analyzedWidth = (maxX-minX);
        analyzedHeight = (maxY-minY);
        analyzedPixels = (analyzedWidth * analyzedHeight); // if analyzing every pixel

        // if something goes wrong with vision process, not detected will be returned
        Bitmap ret = null;
        try {
            openCamera();
            if (camera == null) {
                return null;
            }
            startCamera();
            if (cameraCaptureSession == null) {
                return null;
            }

            // loop until we receive an image from the camera
            boolean haveBitmap = false;
            while (!haveBitmap) {
                Bitmap bmp = frameQueue.poll().copy(Bitmap.Config.ARGB_8888,true);
                ret = bmp.copy(Bitmap.Config.ARGB_8888,false);
                //Bitmap bmp = Bitmap.createBitmap(frameQueue.poll().copy(Bitmap.Config.ARGB_8888,true),minX,minY,(maxX-minX),(maxY-minY));
                if (bmp != null) {
                    // if we have a frame, run operations and break from the loop
                    onNewFrame(bmp, save);
                    haveBitmap = true;
                    //ret = bmp;
                    bmp.recycle();
                }
            }

        } finally {
            closeCamera();
        }
        return ret;
    }

    //----------------------------------------------------------------------------------------------
    // Bitmap Operations
    //----------------------------------------------------------------------------------------------

    private void onNewFrame(Bitmap frame, boolean save) {
        if (save == true) {
            annotateBitmap(frame);
            saveBitmap(frame);
        }
        //frame.recycle(); // not strictly necessary, but helpful
    }

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
    // Camera Operations
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
