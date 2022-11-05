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


@Autonomous(name="Chester Red Side Blue Terminal", group="Robot")
public class ChesterRedBlueTerminal extends LinearOpMode {

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
        //drive and deliver to the low junction
        rb.driveStraightInches(8,0,.4);
        rb.driveStrafeInches(7,0,-.5);
        sleep(250);
        rb.collectorOpen();
        sleep(250);
        //realign against the wall
        rb.driveStrafeInches(8,0,.5);
        //drive toward the blue junction and turn to face the pile
        rb.driveStraightInches(8,0,-.4);
        rb.turnToAngle(-85,.3);
        //drive towards pile
        rb.driveStraightInches(46,-85,.4);
        rb.lifterCS4();
        rb.driveStrafeInches(34,-85,-.4);
        sleep(250);
        rb.collectorClose();
        sleep(250);
        rb.lifterLow();
        rb.driveStrafeInches(24,-85,.5);
        rb.turnToAngle(0,.3);
        //go towards the high

        rb.driveStraightInches(18,0,.4);
        rb.lifterHigh();
        rb.driveStrafeInches(6,0,-.4);
        sleep(1250);
        rb.collectorOpen();
        sleep(250);
        //strafe back and scoot, then turn towards the pile again
        rb.driveStrafeInches(6,0,.5);
        rb.lifterLow();
        rb.driveStraightInches(10,0,-.4);
        rb.turnToAngle(-85,.3);
        rb.lifterCS3();
        rb.driveStrafeInches(29,-85,-.5);
        sleep(250);
        rb.collectorClose();
        sleep(250);
        rb.lifterLow();
        sleep(250);
        rb.driveStrafeInches(24,-85,.5);
        rb.turnToAngle(0,.3);
        //go towards the high

        rb.driveStraightInches(16,0,.4);
        rb.lifterHigh();
        rb.driveStrafeInches(6,0,-.4);
        sleep(1250);
        rb.collectorOpen();
        sleep(250);
        //strafe back and scoot, then turn towards the pile again
        rb.driveStrafeInches(4,0,.5);
        rb.lifterZero();
        sleep(250);

        switch(3){
            case 1:
                rb.driveStraightInches(12,0,.4);
                break;
            case 2:
                rb.driveStraightInches(8,0,-.4);
                break;
            case 3:
                rb.driveStraightInches(22,0,-.4);
                break;
            case 4:
                rb.driveStraightInches(22,0,-.4);
                break;

        }
















    }


    }
