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

@Autonomous(name="M1Auto", group="Linear Opmode")
public class M1Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        M1Base mb = new M1Base(this);
        mb.init();
        waitForStart();

        //decide which square
        switch(2) {

            case 1:
                // Square A

                //grab that wobble goal!
                mb.driveStraight(5,0,.3);

                //path to square c but at angle
                mb.driveStraight(55, 15, .3);

                //to square c
                mb.turnToAngle(0,.3);
                mb.driveStraight(10, 0, .3);

                //almost to the wobble goal
                mb.driveStraight(69,18,-.3);

                //grab that wobble goal!
                mb.turnToAngle(-50,.3);
                mb.driveStraight(44,-50,.3);

                //to square c
                mb.turnToAngle(66,.2);
                mb.driveStraight(60,55,.3);

                //park
                mb.driveStraight(17,45,-.3);
                mb.turnToAngle(0,.3);
                mb.driveStraight(15, 0, .3);
                sleep(500);
                break;

            case 2:
                //Square C
                //DOWNLOAD FIRST!
                //to the rings
                //grab that wobble goal!
                mb.driveStraight(5,0,.3);

                //path to square c but at angle
                mb.driveStraight(55, 15, .3);

                //to square c
                mb.turnToAngle(0,.3);
                mb.driveStraight(57, 0, .5);

                mb.driveStraight(126,0,-.6);

                //grab that wobble goal!
                mb.turnToAngle(-50,.3);
                mb.driveStraight(44,-50,.3);


                break;

            case 3:
                //Square B

                //to the rings
                mb.driveStraight(50,0,.3);

                //to square b
                mb.driveStraight(40,-12,.3);

                //park
                mb.driveStraight(12,0,-.3);

        }
    }
}
