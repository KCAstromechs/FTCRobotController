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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Coins TeleOp", group="Iterative Opmode")
public class CoinsTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private M1_Robot_Base rb;
    double leftPower;
    double rightPower;
    double lifterPower;
    double intakePower;
    double trigger;





    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        rb = new M1_Robot_Base(hardwareMap, telemetry);


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {



        //lifter
        rb.changeLifterPosition((int)(-gamepad2.right_stick_y*48.0));



        //duck
        if (gamepad2.dpad_up){
            rb.duckON();
        }

        if (gamepad2.dpad_down){
            rb.duckReverse();
        }

        if (gamepad2.dpad_right){
            rb.duckOFF();
        }

        //intake
        if(gamepad2.a){
            rb.setIntakeDischarge();
        }
        if (gamepad2.b){
            rb.setIntakeOff();
        }
        if(gamepad2.y){
            rb.setIntakeCollect();
        }

        //slow button
        if (gamepad1.left_bumper){
            leftPower = -gamepad1.left_stick_y/4;
            rightPower = -gamepad1.right_stick_y/4;
        }
        else{
            //hey, it's negative because up on the joystick is negative, and we need to make sure that the number it returns is positive
            leftPower = -gamepad1.left_stick_y/2;
            rightPower = -gamepad1.right_stick_y/2;
        }
        //tank
        rb.setSidePowers(leftPower, rightPower);

        //strafing
        trigger=gamepad1.right_trigger-gamepad1.left_trigger;
        rb.strafe(trigger);





        rb.performUpdates();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
