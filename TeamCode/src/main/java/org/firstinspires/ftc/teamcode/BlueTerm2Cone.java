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
@Autonomous(name="BLUE Terminal", group="Robot")
public class BlueTerm2Cone extends LinearOpMode {

    public M2RobotBase rb;
    public double timeRemainingAfterVision;
    private VisionBase vision;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        rb = new M2RobotBase(hardwareMap,telemetry);
        vision = new VisionBase(hardwareMap, telemetry);

        vision.initVision();
        rb.collectorClose();

        telemetry.addData("ready?", "ready");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        VisionBase.COLOR color = vision.findRGB(200,280,225,395, true);
        timeRemainingAfterVision = getRuntime();
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

        //-------------------------------------------------------------------------------
        // THE RELOCATION OF THE SIGNAL CONE
        //-------------------------------------------------------------------------------

        rb.driveStrafeInches(7,0,.4);
        rb.driveStraightInches(25,0,.3);
        rb.turnToAngle(-65,.2);
        rb.driveStraightInches(3,-65,.3);
        rb.driveStraightInches(3,-65,-.3);
        sleep(250);
        rb.lifterMedium();
        rb.turnToAngle(37,.3);
        sleep(1500);
        rb.driveStraightInches(4,37,.3);
        sleep(500);
        rb.scootLifterDown();
        sleep(250);
        rb.collectorOpen();
        rb.driveStraightInches(9,37,-.4);
        rb.lifterCS5();
        //-------------------------------------------------------------------------------
        // second cone
        //-------------------------------------------------------------------------------
        rb.turnToAngle(-85,.3);
        rb.driveStrafeInches(40,-85,.4);
        rb.driveStraightInches(13,-85,.3);
        rb.colorSensorDetect(false,-85, true, 400);
        rb.coneDrive(10,-85,.3);
        sleep(250);
        rb.collectorClose();
        sleep(250);
        rb.lifterMedium();
        sleep(250);
        rb.driveStraightInches(19,-85,-.4);
        rb.turnToAngle(90,-.3);

        //tune me! im an annoyance!
        rb.driveStrafeInches(21,90,.4);
        rb.driveStraightInches(4,90,.3);
        //i am no longer annoying, do not tune me
        sleep(250);
        rb.scootLifterDown();
        sleep(250);
        rb.collectorOpen();
        sleep(250);
        rb.lifterHigh();
        rb.driveStraightInches(4,90,-.3);
        rb.turnToAngle(0,.3);






        switch(color){
            case BLUE:
                rb.driveStraightInches(11,0,-.4);
                rb.driveStrafeInches(32,0,-.5);



                break;

            case GREEN:
            case NOT_DETECTED:


                break;

            case RED:
                rb.driveStraightInches(11,0,-.4);
                rb.driveStrafeInches(30,0,.5);
                break;
        }

        rb.collectorClose();
        rb.lifterZero();
        rb.turnToAngle(0,0.2);
        sleep(5000);





    }


        }


