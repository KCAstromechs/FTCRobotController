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


@Autonomous(name="Chester Red Side Red Terminal", group="Robot")
public class ChesterRed extends LinearOpMode {

    public M1_Robot_Base rb;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        VisionBase vision = new VisionBase(hardwareMap, telemetry);

        // do this before match start
        vision.initVision();
        rb = new M1_Robot_Base(hardwareMap, telemetry);


        rb.collectorClose();
        telemetry.addData("status:", "ready");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        VisionBase.COLOR color = vision.findRGB(235,260,255,350, true);

        if (color == VisionBase.COLOR.RED) {
            telemetry.addData("Final Answer", "RED");
        }
        else if (color == VisionBase.COLOR.GREEN) {
            telemetry.addData("Final Answer", "GREEN");
        }
        else if (color == VisionBase.COLOR.BLUE) {
            telemetry.addData("Final Answer", "BLUE");
        }
        else {
            telemetry.addData("Final Answer", "NOT DETECTED");
        }
        telemetry.update();

        sleep(1000);

        rb.lifterLow();
        //drive and deliver to the low junction
        rb.driveStraightInches(17,0,-.3);
        rb.driveStrafeInches(7,0,-.4);
        sleep(500);
        rb.scootLifterDown();
        sleep(250);
        rb.collectorOpen();
        sleep(250);
        rb.lifterLow();
        sleep(500);
        rb.driveStrafeInches(7,0,.4);
        //drive toward the blue junction and turn to face the pile
        rb.driveStraightInches(9,0,.3);
        rb.turnToAngle(95,.3);
        //drive towards pile
        rb.driveStraightInches(48,95,-.3);
        rb.lifterCS4();
        rb.driveStrafeInches(38,95,-.4);
        sleep(250);
        rb.collectorClose();
        sleep(250);
        rb.lifterLow();
        rb.driveStrafeInches(29,95,.4);
        rb.turnToAngle(0,.3);
        //go towards the high

        rb.driveStraightInches(15,0,-.3);
        rb.lifterHigh();
        sleep(2000);
        rb.driveStrafeInches(4,0,-.4);
        sleep(1250);
        rb.scootLifterDown();
        sleep(500);
        rb.collectorOpen();
        sleep(250);
        rb.lifterHigh();
        //strafe back and scoot, then turn towards the pile again
        rb.driveStrafeInches(5,0,.4);

        rb.lifterZero();
        sleep(250);


        switch(color){
            case RED:
                rb.driveStraightInches(4,0,-.4);
                break;
            case GREEN:
            case NOT_DETECTED:
                rb.driveStraightInches(5,0,.4);
                break;
            case BLUE:
                rb.driveStraightInches(29,0,.4);
                break;

        }

        sleep(2000);















    }


}

