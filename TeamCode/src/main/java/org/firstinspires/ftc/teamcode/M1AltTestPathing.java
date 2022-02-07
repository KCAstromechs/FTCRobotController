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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
@Autonomous(name="RED Delivery 2 block test")
public class M1AltTestPathing extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Alt_M1_Robot_Base rb;

    @Override
    public void runOpMode() throws InterruptedException {
        rb = new Alt_M1_Robot_Base(hardwareMap, telemetry);


        //sleep to let the gyro initialize and chill
        sleep(500);
        telemetry.addData("ready","ready");
        telemetry.update();


        waitForStart();

        // A IS CLOSEST TO THE WAREHOUSE ON BLUE

        rb.setDriveReadyLifter();
        sleep(1000);
        switch (1) {
            case 1:
              /*  telemetry.addData("Final Answer", "LEFT");
                rb.setDriveReadyLifter();
                //get away from starting position
                rb.driveStraightInches(4,0,-.5);
                rb.driveStrafeInches(5,0,.6);
                //turn to fit in between the tse and barrier
                rb.turnToAngle(60,.4);
                rb.driveStraightInches(3,60,.5);
                //turn in the direction to deliver the block
                rb.turnToAngle(85,.4);
                rb.driveStraightInches(26,85, .5);
                //strafe to the shipping hub
                rb.driveStrafeInches(7,85,.4);
                //strafe away from the hub and move to position to get into the warehouse
                rb.driveStrafeInches(7,85,-.4);
                rb.driveStraightInches(26,85,-.6);
                rb.turnToAngle(0,.5);
                //go into the warehouse
                //rb.driveStraightInches();



             //   rb.setIntakeDischarge();
                sleep(1500);
              //  rb.setIntakeOff();

               */
                telemetry.addData("Final Answer", "LEFT");
                rb.setDriveReadyLifter();
                rb.driveStraightInches(38,0,-.5);
                rb.driveStrafeInches(24,0,.6);
                //rb.setIntakeDischarge();
                sleep(1500);
               // rb.setIntakeOff();
                rb.driveStrafeInches(22,0,-.6);
                //into the warehouse


                //risky stuff idk
                //will be in all programs
                rb.driveStraightInches(64,0,.5);
                rb.driveStrafeInches(2,0,.6);
                rb.turnToAngle(-90,.4);
                rb.setLifterO();
                rb.driveStrafeInches(4,-90,.6);
                //rb.setIntakeOn();
                //sleep(1500);
               // rb.setIntakeOff();
                rb.driveStrafeInches(4,-90,-.6);
                rb.setDriveReadyLifter();
                rb.turnToAngle(0,.4);
                rb.driveStrafeInches(2,0,-.6);
                rb.driveStraightInches(64,0,-.5);
                //


                //same delivery as first time
                rb.driveStraightInches(38,0,-.5);
                rb.driveStrafeInches(24,0,.6);
                //rb.setIntakeDischarge();
                sleep(1500);
                // rb.setIntakeOff();
                rb.driveStrafeInches(22,0,-.6);






                break;

            case 2:
                telemetry.addData("Final Answer", "CENTER");
                rb.setLifterLevel2();


                rb.driveStraightInches(4,0,-.5);
                rb.driveStrafeInches(5,0,.6);
                rb.turnToAngle(30,.4);
                rb.driveStrafeInches(25,30,.5);
               // rb.setIntakeDischarge();
                sleep(1500);
              //  rb.setIntakeOff();

                break;

            case 3:
                telemetry.addData("Final Answer", "RIGHT");
                rb.setLifterLevel3();
                //delete in actual code idk
                rb.driveStraightInches(4,0,-.5);
                rb.driveStrafeInches(5,0,.6);
                rb.turnToAngle(90,.4);
                rb.driveStraightInches(20,90,.5);
               // rb.setIntakeDischarge();
                sleep(1500);
               // rb.setIntakeOff();
                //strafe away
                break;
        }
        rb.setDriveReadyLifter();
        telemetry.update();











    }
    }

