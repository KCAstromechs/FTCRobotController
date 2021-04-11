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
import java.util.ServiceLoader;

@Autonomous(name = "M3Auto", group = "Linear Opmode")
public class M3Auto extends LinearOpMode {

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
        M2Base mb = new M2Base(this);
        mb.init();

        //turn on vision
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(3, 16.0 / 9.0);
        }
        //sleep(2000);
        telemetry.addData("ready?", "ok");
        telemetry.update();
        waitForStart();

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
        }


        mb.wobbleServoPosition("CLOSED");
        //drive forwa
        mb.driveStraight(47, 0, .4);
        //shooter
        mb.shooter(-.75);
        mb.turnToAngle(21, .4);
        mb.driveStraight(2, 21, .5);
        sleep(500);


        //r1
        mb.rotator(-1);
        sleep(400);
        mb.rotator(0);
        //r2
        sleep(600);
        mb.rotator(-1);
        sleep(400);
        mb.rotator(0);
        //r3
        sleep(600);
        mb.rotator(-1);
        sleep(400);
        mb.rotator(0);
        //off
        mb.rotator(0);
        mb.shooter(0);





        //decide square
        // square a= none square b = one square c = quad
        switch (ringCount) {

            case "None":
                // Square A

                //Ndone
                //drive forward and deliver the wobble goal
                mb.driveStraight(6, 21, .5);
                //bring mech down
                mb.wobbleGoalMech(.7, 3500);
                //moving toward the square
                mb.turnToAngle(-104, .4);
                mb.driveStraight(17, -104, -.5);
                mb.wobbleGoalMech(.8,3800);
                sleep(500);
                mb.wobbleServoPosition("OPEN");
                sleep(500);
                //back up and begin driving toward the second wobble goal
                mb.driveStraight(7,-90, .5);
                mb.turnToAngle(-13.5, .4);
                mb.driveStraight(38,-13.5,-.5);
                //collect and begin to deliver
                mb.wobbleServoPosition("CLOSED");
                sleep(800);
                mb.wobbleGoalMech(.8,2000);
                sleep(1000);
                mb.turnToAngle(-160,.4, true);
                mb.wobbleGoalMech(.8,3300);
                mb.driveStraight(33, -160, -.5);
                mb.wobbleGoalMech(.8,3800);
                sleep(1000);
                mb.wobbleServoPosition("OPEN");
                sleep(750);
                mb.wobbleGoalMech(.8,10);
                mb.driveStraight(3, -160, .5);
                mb.turnToAngle(-45, .4,true);
                mb.driveStraight(17,-45, .5);
                break;

            case "Quad":
                //Square C
                //tuned
                mb.turnToAngle(-150,.4);
                mb.wobbleGoalMech(.8,3500);
                mb.driveStraight(58,-150, -.5);
                mb.wobbleGoalMech(.8,3800);
                sleep(500);
                mb.wobbleServoPosition("OPEN");
                sleep(500);
                mb.driveStraight(5,-150, .5);
                //go grab the other one
                mb.turnToAngle(1,.4);
                mb.driveStraight(60, 0, -.7);
                mb.driveStraight(9, 0, -.4);
                mb.wobbleServoPosition("CLOSED");
                sleep(1000);
                mb.wobbleGoalMech(.7, 2000);
                sleep(800);
                //go to deliver
                mb.turnToAngle(-177, .4, true);
                mb.wobbleGoalMech(.8,3000);
                mb.driveStraight(75, -177, -.7 );
                //deliver
                mb.wobbleGoalMech(.7, 3800);
                sleep(500);
                mb.wobbleServoPosition("OPEN");
                sleep(500);
                //park
                mb.wobbleGoalMech(.7, 10);
                mb.driveStraight(17, -180, .6);
                break;

            case "Single":
                //Square B
                mb.turnToAngle(-155, .4);
                mb.wobbleGoalMech(.8,3500);
                mb.driveStraight(27, -155, -.5);
                mb.wobbleGoalMech(.8,3800);
                sleep(800);
                mb.wobbleServoPosition("OPEN");
                sleep(500);
                //return for the second one
                mb.driveStraight(32,-155,.6);
                mb.turnToAngle(-160, .4);
                mb.driveStraight(10, -150, -.5);
                mb.wobbleServoPosition("CLOSED");
                sleep(750);
                mb.wobbleGoalMech(.8, 2000);
                //
                mb.turnToAngle(174, .4);
                mb.wobbleGoalMech(.8,3300);
                mb.driveStraight(48, 174, -.6);
                //deliver
                mb.wobbleGoalMech(.8,3800);
                sleep(500);
                mb.wobbleServoPosition("OPEN");
                sleep(500);
                //park
                mb.wobbleGoalMech(.8, 10);
                mb.driveStraight(3, 174, .6);


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
