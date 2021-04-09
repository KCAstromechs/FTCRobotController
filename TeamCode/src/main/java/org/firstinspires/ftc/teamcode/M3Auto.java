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
        mb.turnToAngle(21, .4);
        mb.driveStraight(2, 21, .5);
        sleep(500);

        // shooter power up and rotator goes

        mb.shooter(-.75);
        sleep(2000);
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
        switch ("Single") {

            case "None":
                // Square A

                //NEEDS TUNING
                //drive forward and deliver the wobble goal
                mb.driveStraight(6, 21, .5);
                mb.turnToAngle(-104, .4);
                mb.driveStraight(17, -104, -.5);
                //deliver
                mb.wobbleGoalMech(.7, 4000);
                sleep(1500);
                mb.wobbleServoPosition("OPEN");
                sleep(500);
                //back up and begin driving toward the second wobble goal
                mb.driveStraight(7,-90, .5);
                mb.turnToAngle(-15, .4);
                mb.driveStraight(43,-15,-.5);
                //collect and begin to deliver
                mb.wobbleServoPosition("CLOSED");
                sleep(1500);
                mb.wobbleGoalMech(.7,2000);
                sleep(2000);
                mb.turnToAngle(-160,.4, true);
                mb.driveStraight(28, -160, -.5);

                //deliver
                mb.wobbleGoalMech(.7,4000);
                sleep(1000);
                mb.wobbleServoPosition("OPEN");
                sleep(750);
                mb.wobbleGoalMech(.7,0);
                mb.driveStraight(10,-160, -.5);





                break;

            case "Quad":
                //Square C
                //TUNE THIS
                mb.turnToAngle(-150,.4);
                mb.driveStraight(58,-150, -.5);
                mb.wobbleGoalMech(.7,4000);
                sleep(1500);
                mb.wobbleServoPosition("OPEN");
                sleep(750);
                mb.turnToAngle(4,.4);
                mb.driveStraight(60, 4, -.7);
                mb.driveStraight(12, 4, -.4);
                mb.wobbleServoPosition("CLOSED");
                sleep(1000);
                mb.wobbleGoalMech(.7, 2000);
                sleep(1000);
                mb.turnToAngle(-170, .4, true);
                mb.driveStraight(70, -177, -.7 );
                mb.wobbleGoalMech(.7, 4000);
                sleep(1500);
                mb.wobbleServoPosition("OPEN");
                sleep(750);
                mb.wobbleGoalMech(.7, 0);
                mb.driveStraight(38, -180, .6);





                break;

            case "Single":
                //Square B
                mb.turnToAngle(-155, .4);
                mb.driveStraight(6, -155, -.5);
                mb.wobbleGoalMech(.7,4000);
                sleep(1500);
                mb.wobbleServoPosition("OPEN");
                sleep(750);
                mb.turnToAngle(-35, -.4);
                mb.driveStraight(60, -35, -.5);
                mb.wobbleServoPosition("CLOSED");
                sleep(750);
                mb.wobbleGoalMech(.7, 2000);
                //sleeo?
                mb.turnToAngle(170, .4);
                mb.driveStraight(60, 175, -.6);
                mb.wobbleGoalMech(.7,4000);
                sleep(1500);
                mb.wobbleServoPosition("OPEN");
                sleep(750);
                mb.wobbleGoalMech(.7, 0);
                mb.driveStraight(12, -5, .6);


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
