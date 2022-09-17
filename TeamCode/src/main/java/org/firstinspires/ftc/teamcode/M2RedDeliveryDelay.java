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


@Autonomous(name="RED Preload Delivery (DELAY)")
public class M2RedDeliveryDelay extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private M2_Robot_Base rb;
    private VisionBase vision;

    @Override
    public void runOpMode() throws InterruptedException {
        rb = new M2_Robot_Base(hardwareMap, telemetry);
        VisionBase vision = new VisionBase(hardwareMap, telemetry);
        vision.initVision();
        //sleep to let the gyro initialize and chill
        sleep(500);
        telemetry.addData("ready","ready");
        telemetry.update();


        waitForStart();


        //VisionBase.TSEPosition position = vision.findTSEPosition(85,530,80,190, true);
        VisionBase.TSEPosition position = vision.findTSEPosition(20,620,20,460, false);

        rb.setDriveReadyLifter();
        sleep(10000); //5000
        // A IS CLOSEST TO THE WAREHOUSE ON BLUE
        switch (position) {
            case LEFT:
                telemetry.addData("Final Answer", "LEFT");
                rb.setDriveReadyLifter();
                rb.driveStraightInches(24,0,.6);
                rb.driveStrafeInches(29,0,.8);
                rb.setIntakeDischarge();
                sleep(1500);
                rb.setIntakeOff();
                rb.driveStrafeInches(11,0,-.8);

                break;

            case CENTER:
                telemetry.addData("Final Answer", "CENTER");
                rb.setLifterLevel2();
                rb.driveStraightInches(24,0,.6);
                rb.driveStrafeInches(30,0,.8);
                rb.setIntakeDischarge();
                sleep(1500);
                rb.setIntakeOff();
                rb.driveStrafeInches(12,0,-.8);
                break;

            case RIGHT:
                telemetry.addData("Final Answer", "RIGHT");
                rb.setLifterLevel3();
                rb.driveStraightInches(24,0,.6);
                rb.driveStrafeInches(35,0,.8);
                rb.setIntakeDischarge();
                sleep(1500);
                rb.setIntakeOff();
                rb.driveStrafeInches(15,0,-.8);

                break;

            case NOT_DETECTED:
                telemetry.addData("Final Answer", "NOT DETECTED");
                rb.setLifterLevel3();
                rb.driveStraightInches(24,0,.6);
                rb.driveStrafeInches(35,0,.8);
                rb.setIntakeDischarge();
                sleep(1500);
                rb.setIntakeOff();
                rb.driveStrafeInches(15,0,-.8);
                break;
        }

        telemetry.update();





        //move away from the cake

        rb.driveStraightInches(41,0,-.6);
        rb.setDriveReadyLifter();
        rb.turnToAngle(40,.4);
        rb.driveStraightInches(5,40,-.6);
        rb.deliverDuck(false, -.1);
        //back up and move to storage unit
        rb.driveStraightInches(3,40,.6);
        rb.turnToAngle(0,.4);
        rb.driveStrafeInches(17,0,.8);
        rb.driveStraightInches(4,0,-.6);
        rb.setLifterO();
        sleep(1500);
    }
    }

