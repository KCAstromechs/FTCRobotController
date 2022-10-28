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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="Chester Red", group="Robot")
public class ChesterRed extends LinearOpMode {

    public M1_Robot_Base rb;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        rb = new M1_Robot_Base(hardwareMap, telemetry);

        rb.collectorClose();
        telemetry.addData("status:", "ready");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        rb.lifterLow();
        rb.driveStraightInches(8, 0, .4);
        rb.driveStrafeInches(45,0,-.4);
        sleep(250);
        rb.collectorOpen();
        sleep(250);
        rb.driveStrafeInches(3,0,.4);
        rb.driveStraightInches(7,0,.4);
        rb.lifterCS4();
        rb.turnToAngle(90,.3);
        rb.driveStraightInches(22,90,-.4);
        rb.driveStrafeInches(7,90,-.5);
        rb.collectorClose();
        sleep(250);
        rb.lifterLow();
        rb.driveStrafeInches(11,90,.5);
        rb.turnToAngle(-90,-.3);
        rb.driveStrafeInches(21,-90,-.5);
        rb.lifterHigh();
        rb.driveStraightInches(11,-90,.4);
        sleep(250);
        rb.driveStrafeInches(2.5,-90,-.4);
        sleep(1500);
        rb.collectorOpen();
        sleep(250);
        rb.driveStrafeInches(2.5,-90,.4);
        rb.lifterCS3();
        rb.driveStraightInches(10,-90,-.4);
        rb.turnToAngle(90,.4);
        rb.driveStrafeInches(35,90,-.4);
        rb.collectorClose();
        sleep(250);
        rb.lifterLow();
        rb.driveStrafeInches(10,90,.4);
        rb.turnToAngle(-90,-.4);
        rb.driveStrafeInches(22,-90,-.4);
        rb.lifterHigh();
        rb.driveStraightInches(9,-90,.4);
        sleep(250);
        rb.driveStrafeInches(3,-90,-.4);
        sleep(1000);
        rb.collectorOpen();
        sleep(250);
        rb.driveStrafeInches(2,-90,.4);
        rb.driveStraightInches(8,-90,-.4);
        rb.lifterZero();
        rb.turnToAngle(0,.3);

        switch(2){


            case 1:
            rb.driveStraightInches(16,0,.4);
            break;

            case 2:
                rb.driveStraightInches(4,0,.4);
            break;

            case 3:
            rb.driveStraightInches(14,0,-.4);
            break;



        }





    }


    }

