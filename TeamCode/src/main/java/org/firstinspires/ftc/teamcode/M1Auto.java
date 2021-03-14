/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="M1Auto", group="Linear Opmode")
public class M1Auto extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "Ac8xsqH/////AAAAGcG2OeE2NECwo7mM5f9KX1RKmDT79NqkIHc/ATgW2+loN9Fr8fkfb6jE42RZmiRYeei1FvM2M3kUPdl53j" +
            "+oeuhahXi7ApkbRv9cef0kbffj+4EkWKWCgQM39sRegfX+os6PjJh1fwGdxxijW0CYXnp2Rd1vkTjIs/cW2/7TFTtuJTkc17l" +
            "+FNJAeqLEfRnwrQ0FtxvBjO8yQGcLrpeKJKX/+sN+1kJ/cvO345RYfPSoG4Pi+wo/va1wmhuZ/WCLelUeww8w8u0douStuqcuz" +
            "ufrsWmQThsHqQDfDh0oGKZGIckh3jwCV2ABkP0lT6ICBDm4wOZ8REoyiY2kjsDnnFG6cT803cfzuVuPJl+uGTEf";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private String ringCount = "None";

    @Override
    public void runOpMode() throws InterruptedException {
        M1Base mb = new M1Base(this);
        mb.init();

        // make sure we're legal
        mb.wobbleGoalPosition(1);
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }
        waitForStart();
        telemetry.addData("encoderY", mb.encoderY.getCurrentPosition());
        //grab that wobble goal!
        mb.driveStraight(5, 0, .3);
        telemetry.update();
        //path to square c but at angle
        mb.driveStraight(30, 15, .3);
        telemetry.update();
        //robot is stopped
        //TODO CHANGE WHEN COMPLETE - BACK TO 1000
        sleep(2000);

        //PUT THIS CODE WHERE YOU WANT TO RUN TENSORFLOW
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                     ringCount = recognition.getLabel();
                }
                telemetry.update();

            }
        }


        //check ring count
        telemetry.addData("ring count", ringCount);
        telemetry.update();

        if (tfod != null) {
            tfod.shutdown();


// square a= none square b = one square c = quad
            //decide which square
            switch (ringCount) {

                case "None":
                    // Square A  - FULLY TUNED

                    //path to square but at angle
                    mb.driveStraight(25, 15, .3);

                    //to square
                    mb.turnToAngle(0, .3);
                    mb.driveStraight(8, 0, .3);

                    //almost to the wobble goal
                    mb.driveStraight(67, 18, -.3);

                    //grab that wobble goal!
                    mb.turnToAngle(-50, .3);
                    mb.driveStraight(44, -50, .3);

                    //to square
                    mb.turnToAngle(66, .2);
                    mb.driveStraight(56, 55, .3);

                    //park
                    mb.driveStraight(17, 45, -.3);
                    mb.turnToAngle(0, .3);
                    mb.driveStraight(15, 0, .3);
                    sleep(500);
                    break;

                case "Quad":
                    //Square C  FULLY TUNED

                    //path to square but at angle
                    mb.driveStraight(25, 15, .3);

                    //to square
                    mb.turnToAngle(0, .3);
                    mb.driveStraight(57, 0, .5);

                    mb.driveStraight(105, 0, -.6);

                    //grab that wobble goal!
                    mb.turnToAngle(-60, .3);
                    mb.driveStraight(48, -65, .3);

                    //towards the field center and around the rings
                    mb.turnToAngle(25, .2);
                    mb.driveStraight(100, 25, .4);

                    //park
                    mb.driveStraight(8, 25, -.4);
                    mb.turnToAngle(0, .3);
                    mb.driveStraight(27, 0, -.4);

                    break;

                case "Single":
                    //Square B

                    // turn to square B and drive with the preload wobble goal
                    mb.turnToAngle(-20,.3);
                    mb.driveStraight(55,-20,.4);

                    //back towards the camera analyze point
                    mb.driveStraight(55,-20,-.5);
                    mb.turnToAngle(0,.3);

                    //back towards wall  CHANGED
                    mb.driveStraight(20,0,-.5);

                    //turn towards the wobble goal
                    mb.turnToAngle(-60, .3);
                    mb.driveStraight(48, -65, .3);

                    //turn towards square b and go around the rings CHANGED
                    mb.turnToAngle(20, .2);
                    mb.driveStraight(67,20,.4);

                    //reverse and park
                    mb.driveStraight(6,25,-4);
                    mb.turnToAngle(0,.3);

            }

        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
