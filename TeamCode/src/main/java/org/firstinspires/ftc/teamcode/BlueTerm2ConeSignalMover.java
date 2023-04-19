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
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="BLUE Terminal BLUE Tape", group="Robot")
public class BlueTerm2ConeSignalMover extends LinearOpMode {

    public M2RobotBase rb;
    public double timeRemainingAfterVision;
    private ColorVisionBase vision;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        rb = new M2RobotBase(hardwareMap,telemetry);
        vision = new ColorVisionBase(hardwareMap, telemetry);

        vision.initVision();
        rb.collectorClose();

        telemetry.addData("ready?", "ready");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        ColorVisionBase.ZONE zone = vision.findZone(215,280,250,370, false);




        try {


            timeRemainingAfterVision = getRuntime();
            if (zone == ColorVisionBase.ZONE.ONE) {
                telemetry.addData("Final Answer", "ONE");
            } else if (zone == ColorVisionBase.ZONE.TWO) {
                telemetry.addData("Final Answer", "TWO");
            } else if (zone == ColorVisionBase.ZONE.THREE) {
                telemetry.addData("Final Answer", "THREE");
            } else {
                telemetry.addData("Final Answer", "NOT DETECTED");
            }
            telemetry.update();

            //-------------------------------------------------------------------------------
            // THE RELOCATION OF THE SIGNAL CONE
            //-------------------------------------------------------------------------------

            rb.driveStraightInches(1, 0, .3,1500);
            rb.lifterLow();
            rb.driveStrafeInches(35, 0, -.4, 4000);
            rb.driveStraightInches(27, 0, .4, 3000);
            rb.turnToAngle(85, .3);
            rb.horizontalJunctionDistanceDetect(false, 90, 865, .4);
            sleep(250);
            rb.horizontalJunctionDistanceDetect(true, 90, 221, .3);
            sleep(250);
            rb.forwardJunctionDistanceDetect(444);
            sleep(250);
            rb.scootLifterDown();
            sleep(500);

            rb.collectorOpen();
            sleep(250);
            rb.driveStraightInches(2, 90, -.3, 1500);
            rb.turnToAngle(-85, .3);
            rb.driveStraightInches(3, -85, -.2, 1500);
            rb.colorSensorDetect(true, -85, true, 3000);
            rb.lifterCS5();
            rb.coneDrive(4, -85, .3);
            rb.collectorClose();
            //-------------------------------------------------------------------------------
            // second cone
            //-------------------------------------------------------------------------------
            sleep(500);
            rb.lifterMedium();
            sleep(250);
            rb.driveStraightInches(22, -85, -.4, 3000);
            rb.turnToAngle(90, -.3);

            rb.horizontalJunctionDistanceDetect(true, 95, 3000, .4);
            sleep(250);
            rb.horizontalJunctionDistanceDetect(false, 95, 500, .3);
            sleep(250);
            rb.forwardJunctionDistanceDetect(140);
            sleep(250);
            rb.scootLifterDown();
            sleep(250);
            rb.collectorOpen();
            sleep(250);
            rb.lifterHigh();
            rb.driveStraightInches(3, 95, -.3, 1500);
            rb.turnToAngle(5, .3);


            switch (zone) {
                case THREE:
                    rb.driveStraightInches(11, 5, -.4, 3000);
                    rb.driveStrafeInches(32, 5, -.5, 4000);


                    break;

                case TWO:
                case NOT_DETECTED:


                    break;

                case ONE:
                    rb.driveStraightInches(11, 5, -.4, 3000);
                    rb.driveStrafeInches(35, 5, .5, 4000);
                    break;
            }

            rb.collectorClose();
            rb.lifterZero();
            rb.turnToAngle(5, 0.2);
            sleep(5000);


        }catch (DriveTimeoutException dte){
            rb.motorShutdown();
        }


    }


        }


