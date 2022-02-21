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


@Autonomous(name="BLUE DOUBLE Delivery")
public class M2BlueDoubleDeliveryWarehouse extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private M2_Robot_Base rb;

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




        VisionBase.TSEPosition position = vision.findTSEPosition(85,566,84,213, false);
        // A IS CLOSEST TO THE WAREHOUSE ON BLUE

        rb.setDriveReadyLifter();
        sleep(500);
        switch (position) {
            case LEFT:
                telemetry.addData("Final Answer", "LEFT");
                rb.setDriveReadyLifter();
                //move to the hub
                rb.driveStraightInches(33,0,-.5);
                rb.driveStrafeInches(32,0,.6);
                //deliver
                rb.setIntakeDischarge();
                sleep(1500);
                rb.setIntakeOff();
                //drive back to the wall
                rb.driveStrafeInches(30,0,-.6);




                break;

            case CENTER:
                telemetry.addData("Final Answer", "CENTER");
                rb.setLifterLevel2();

                //line up and move towards the lifter
                rb.driveStraightInches(33,0,-.4);
                rb.driveStrafeInches(32,0,.6);
                rb.setIntakeDischarge();
                sleep(1500);
                rb.setIntakeOff();
                rb.driveStrafeInches(30,0,-.6);

                break;

            case RIGHT:
                telemetry.addData("Final Answer", "RIGHT");
                rb.setLifterLevel3();

                //line up and move towards the lifter
                rb.driveStraightInches(33,0,-.4);
                rb.driveStrafeInches(37,0,.6);
                rb.setIntakeDischarge();
                sleep(1500);
                rb.setIntakeOff();
                //strafe away
                rb.driveStrafeInches(35,0,-.6);

               //insert new code here


                break;

            case NOT_DETECTED:
                telemetry.addData("Final Answer", "NOT DETECTED");
                rb.setLifterLevel3();

                //line up and move towards the lifter
                rb.driveStraightInches(33,0,-.4);
                rb.driveStrafeInches(37,0,.6);
                rb.setIntakeDischarge();
                sleep(1500);
                rb.setIntakeOff();
                //strafe away
                rb.driveStrafeInches(35,0,-.6);
                //insert new code here



                break;
        }

        //TODO: exist
        //drive into the warehouse
        rb.driveStraightInches(62,0,.5);
        rb.driveStrafeInches(8,0,.6);
        //turn in the direction of the freight
        rb.setLifterO();
        sleep(500);
        rb.turnToAngle(90,.4);
        //collect
        rb.setIntakeCollect();
        rb.driveStrafeInches(11,90,.6);
        sleep(1500);
        rb.setIntakeOff();
        //back up from freight piles and turn away
        rb.driveStrafeInches(9,90,-.6);
        rb.setLifterLevel3();
        rb.turnToAngle(0,.4);
        //move into position for delivering the block
        rb.driveStrafeInches(8,0,-.6);
        rb.driveStraightInches(62, 0,-.5);

        rb.driveStrafeInches(36,0,.6);
        rb.setIntakeDischarge();
        sleep(2000);
        rb.setIntakeOff();
        //strafe away
        rb.driveStrafeInches(38,0,-.6);

        rb.setDriveReadyLifter();
        telemetry.update();



        rb.driveStraightInches( 62,0,.5);
        sleep(1500);
        rb.setLifterO();
        sleep(500);











    }
    }

