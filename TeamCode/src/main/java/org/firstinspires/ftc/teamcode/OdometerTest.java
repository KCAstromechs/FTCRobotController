package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

    @TeleOp(name="OdometerTest", group="Iterative Opmode")
    public class OdometerTest extends OpMode
    {
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor encoder1 = null;
        private DcMotor encoder2 = null;

        @Override
        public void init() {
            telemetry.addData("Status", "Initialized");
            encoder1 = hardwareMap.get(DcMotor.class, "encoder1");
            encoder2 = hardwareMap.get(DcMotor.class, "encoder2");

            // RESET ENCODERS
            encoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // START THE ENCODERS
            encoder1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encoder2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");


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
            telemetry.addData("encoder1", encoder1.getCurrentPosition());
            telemetry.addData("encoder2", encoder2.getCurrentPosition());
            telemetry.update();
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
        }

    }
