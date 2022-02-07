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




@Autonomous(name="Purse Bot Test Program")
public class M2TestProgram extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private M2_Robot_Base rb;

    @Override
    public void runOpMode() throws InterruptedException {
        rb = new M2_Robot_Base(hardwareMap, telemetry);

        //sleep to let the gyro initialize and chill
        sleep(500);
        telemetry.addData("ready","ready");
        telemetry.update();


        waitForStart();
        telemetry.addData("lifter:", "level 1");
        telemetry.update();
        //lifter movement
        rb.setDriveReadyLifter();
        sleep(3000);

        telemetry.addData("lifter:", "level 2");
        telemetry.update();
        rb.setLifterLevel2();
        sleep(3000);

        telemetry.addData("lifter:", "level 3");
        telemetry.update();
        rb.setLifterLevel3();
        sleep(3000);

        telemetry.addData("lifter:", "level 0, ground level");
        telemetry.update();
        rb.setLifterO();
        sleep(3000);

        telemetry.addData("lifter:", "level 1");
        telemetry.update();
        //lifter movement
        rb.setDriveReadyLifter();
        sleep(3000);


       //intake movement
        rb.TestSetIntakeCollect();
        sleep(1500);
        rb.TestSetIntakeOff();
        sleep(1500);
        rb.TestSetIntakeDischarge();
        sleep(1500);
        rb.setIntakeOff();


        //carousel movement
        rb.TestDuckON();
        sleep(1500);
        rb.TestDuckOFF();
        sleep(1500);
        rb.TestDuckReverse();
        sleep(1500);

        //driveStraight
        telemetry.addData("driving status:", "driveStraight Function, forward, 24 in");
        telemetry.update();
        sleep(3000);
        rb.driveStraightInches(24,0,.4);

        telemetry.addData("driving status:", "driveStraight Function, reverse, 24 in");
        telemetry.update();
        sleep(3000);
        rb.driveStraightInches(24,0,-.4);



        //turnToAngle
        telemetry.addData("driving status:", "turnToAngle Function, 90 degrees");
        telemetry.update();
        sleep(3000);
        rb.turnToAngle(90,.3);

        telemetry.addData("driving status:", "turnToAngle Function, 180/0 degrees");
        telemetry.update();
        sleep(3000);
        rb.turnToAngle(180,.3,true);

        telemetry.addData("driving status:", "turnToAngle Function, -90 degrees");
        telemetry.update();
        sleep(3000);
        rb.turnToAngle(-90,.3);


        telemetry.addData("driving status:", "turnToAngle Function, 0 degrees");
        telemetry.update();
        sleep(3000);
        rb.turnToAngle(0,.3);




        //driveStrafe
        telemetry.addData("driving status:", "driveStrafe Function, forward (lifter side), 24 in");
        telemetry.update();
        sleep(3000);
        rb.driveStrafeInches(24,0,.6);

        telemetry.addData("driving status:", "driveStrafe Function, reverse (non-lifter side), 24 in");
        telemetry.update();
        sleep(3000);
        rb.driveStrafeInches(24,0,-.6);




        //individual motor
        telemetry.addData("driving status:", "individual motor, frontRight, 10 in");
        telemetry.update();
        sleep(3000);
        rb.frontRightTest();

        telemetry.addData("driving status:", "individual motor, frontLeft, 10 in");
        telemetry.update();
        sleep(3000);
        rb.frontLeftTest();

        telemetry.addData("driving status:", "individual motor, backLeft, 10 in");
        telemetry.update();
        sleep(3000);
        rb.backLeftTest();

        telemetry.addData("driving status:", "individual motor, backRight, 10 in");
        telemetry.update();
        sleep(3000);
        rb.backRightTest();


    }
    }

