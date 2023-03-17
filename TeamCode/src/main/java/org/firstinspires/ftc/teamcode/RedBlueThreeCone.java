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
@Autonomous(name="RED Side BLUE Terminal", group="Robot")
public class RedBlueThreeCone extends LinearOpMode {

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

        ColorVisionBase.ZONE zone = vision.findZone(225,300,305,360, true);
        timeRemainingAfterVision = getRuntime();
        if (zone == ColorVisionBase.ZONE.ONE) {
            telemetry.addData("Final Answer", "ONE");
        }
        else if (zone == ColorVisionBase.ZONE.TWO) {
            telemetry.addData("Final Answer", "TWO");
        }
        else if (zone == ColorVisionBase.ZONE.THREE) {
            telemetry.addData("Final Answer", "THREE");
        }
        else {
            telemetry.addData("Final Answer", "NOT DETECTED");
        }
        telemetry.update();

/*
        //-------------------------------------------------------------------------------
        // THE RELOCATION OF THE SIGNAL CONE
        //-------------------------------------------------------------------------------

        rb.driveStrafeInches(10,0,.4);
        rb.driveStraightInches(18,0,.5);
        rb.turnToAngle(-60,.4);
        rb.lifterHigh();
        rb.turnToAngle(0,.3);



        //-------------------------------------------------------------------------------
        // FIRST CONE
        //-------------------------------------------------------------------------------

        //finish the drive to mid field
        rb.driveStraightInches(21,0,.4);
        //turn towards the high
        rb.turnToAngle(49,.3);
        rb.driveStraightInches(4,49,.3);
        sleep(200);
        rb.scootLifterDown();
        sleep(200);
        rb.collectorOpen();
        //raise the roof so it doesn't catch
        sleep(200);
        rb.lifterHigh();
        //prepare for cone pile and cha cha real smooth away
        rb.driveStraightInches(4,49,-.4);
        rb.lifterCS5();





        //------------------------------------------------------------------------------------------
        //  SECOND CONE
        //------------------------------------------------------------------------------------------


        //turn towards pile
        rb.turnToAngle(-85,.3);
        rb.driveStraightInches(24,-85,.4);
        rb.collectorClose();
        sleep(250);
        //lift to be ready for high
        rb.lifterHigh();
        sleep(700);
        rb.driveStraightInches(23,-85,-.4);
        //turn towards the high
        rb.turnToAngle(65,.3);
        rb.driveStraightInches(6,65,.3);
        sleep(200);
        rb.scootLifterDown();
        sleep(200);
        rb.collectorOpen();
        //raise the roof (so it doesn't catch)
        sleep(200);
        rb.lifterHigh();
        //back away
        rb.collectorClose();
        rb.driveStraightInches(6,65,-.4);



        //------------------------------------------------------------------------------------------
        // THIRD CONE
        //------------------------------------------------------------------------------------------

        //prepare for pile
        rb.lifterCS4();
        rb.collectorOpen();
        rb.turnToAngle(-90,.3);
        rb.driveStraightInches(22,-90,.4);
        rb.collectorClose();
        sleep(250);
        rb.lifterHigh();
        sleep(700);
        rb.driveStraightInches(20,-90,-.4);
        rb.turnToAngle(70,.3);
        rb.driveStraightInches(8,70,.3);
        sleep(200);
        rb.scootLifterDown();
        sleep(200);
        rb.collectorOpen();
        //raise the roof (so it doesn't catch)
        sleep(200);
        rb.lifterHigh();
        //back away
        rb.collectorClose();
        rb.driveStraightInches(5,70,-.4);


        rb.lifterZero();
        rb.turnToAngle(7,.3);


        switch(zone){
            case ONE:
                rb.driveStraightInches(3,7,-.5);
                rb.driveStrafeInches(32,7,.5);
                rb.driveStraightInches(6,7,-.5);


                break;

            case TWO:
            case NOT_DETECTED:
                rb.driveStraightInches(6,7,-.5);

                break;

            case THREE:
                rb.driveStraightInches(2,7,-.5);
                rb.driveStrafeInches(27,7,-.5);
                rb.driveStraightInches(6,7,-.5);
                break;
        }

        sleep(2000);


        */
    }


        }


