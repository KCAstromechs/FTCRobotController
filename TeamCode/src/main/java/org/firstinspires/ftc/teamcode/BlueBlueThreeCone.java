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

@Autonomous(name="BLUE Side BLUE Terminal", group="Robot")
public class BlueBlueThreeCone extends LinearOpMode {

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

        VisionBase.COLOR color = vision.findRGB(350,420,250,285, false);
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

        rb.driveStrafeInches(10,0,.4);
        rb.driveStraightInches(18,0,.5);
        rb.turnToAngle(-60,.2);
        rb.lifterHigh();
        rb.turnToAngle(0,.3);



        //-------------------------------------------------------------------------------
        // FIRST CONE
        //-------------------------------------------------------------------------------

        //finish the drive to mid field
        rb.driveStraightInches(21,0,.4);
        //turn towards the high
        rb.turnToAngle(43,.3);
        rb.driveStraightInches(4,43,.3);
        sleep(200);
        // drop him off at school
        rb.scootLifterDown();
        sleep(200);
        rb.collectorOpen();
        sleep(500);
        //raise the roof so it doesn't catch
        rb.lifterHigh();
        sleep(500);
        //prepare for cone pile and cha cha real smooth away
        rb.driveStraightInches(4,43,-.2);
        rb.lifterCS5();
        sleep(1000);





        //------------------------------------------------------------------------------------------
        //  SECOND CONE
        //------------------------------------------------------------------------------------------


        //turn towards pile
        rb.turnToAngle(-80,.3);
        rb.driveStraightInches(30,-80,.4);
        rb.collectorClose();
        sleep(250);
        //lift to be ready for high
        rb.lifterHigh();
        sleep(1000);
        rb.driveStraightInches(25,-80,-.4);
        //turn towards the high
        rb.turnToAngle(55,.3);
        rb.driveStraightInches(5,55,.3);
        sleep(200);
        // drop him off at school
        rb.scootLifterDown();
        sleep(200);
        rb.collectorOpen();
        sleep(500);
        //raise the roof so it doesn't catch
        rb.lifterHigh();
        sleep(500);
        //back away
        rb.collectorClose();
        sleep(200);

/*

        //------------------------------------------------------------------------------------------
        // THIRD CONE
        //------------------------------------------------------------------------------------------

        //prepare for pile
        rb.lifterCS4();
        rb.collectorOpen();
        rb.turnToAngle(-85,.3);
        rb.driveStraightInches(23,-85,.4);
        rb.collectorClose();
        sleep(250);
        rb.lifterHigh();
        sleep(700);
        rb.driveStraightInches(25,-85,-.4);
        rb.turnToAngle(60,.3);
        rb.driveStraightInches(5,60,.3);
        sleep(200);
        rb.collectorOpen();
        //raise the roof (so it doesn't catch)
        sleep(200);
        //back away
        rb.collectorClose();
        rb.driveStraightInches(5,60,-.4);

        //------------------------------------------------------------------------------------------
        // FOURTH CONE
        //------------------------------------------------------------------------------------------

        if (timeRemainingAfterVision > 28){
            telemetry.addData("hello", "i have time for a fourth cone");
            telemetry.update();
        }













        rb.lifterZero();
        rb.turnToAngle(7,.3);
*/

        switch(color){
            case RED:
                rb.driveStraightInches(12,55,-.4);
                rb.turnToAngle(85,.3);
                rb.driveStraightInches(25,85,.3);
                break;

            case GREEN:

            case NOT_DETECTED:
                rb.driveStraightInches(8,55,-.4);
                rb.turnToAngle(0,0.3);
                rb.driveStraightInches(4,0,-0.3);
                break;

            case BLUE:
                rb.driveStraightInches(4,55,-.4);
                rb.turnToAngle(0,0.3);
                rb.driveStrafeInches(30,0,-.4);
                break;
        }
        rb.turnToAngle(0,0.3);
        rb.lifterZero();
        sleep(4000);

    }


        }


