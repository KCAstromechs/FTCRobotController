package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "FieldCentricDrive (Blocks to Java)")
public class FieldCentricDriveM2 extends LinearOpMode {

    private IMU imu_IMU;
    private BNO055IMU imu;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;
        float yawAngle;

        imu_IMU = hardwareMap.get(IMU.class, "imu");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Initialize the IMU.
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        // Prompt user to press start button.
        telemetry.addData("IMU Example", "Press start to continue...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                telemetry.addData("Yaw", "Press Circle or B on Gamepad to reset.");
                // Check to see if reset yaw is requested.
                if (gamepad1.circle) {
                    imu_IMU.resetYaw();
                }
                // Get the orientation and angular velocity.
                yawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).thirdAngle;
                orientation = imu_IMU.getRobotYawPitchRollAngles();
                angularVelocity = imu_IMU.getRobotAngularVelocity(AngleUnit.DEGREES);
                telemetry.addData("Yaw Angle", JavaUtil.formatNumber(yawAngle, 2));
                telemetry.addData("Yaw (Z)", JavaUtil.formatNumber(orientation.getYaw(AngleUnit.DEGREES), 2));
                telemetry.addData("Pitch (X)", JavaUtil.formatNumber(orientation.getPitch(AngleUnit.DEGREES), 2));
                telemetry.addData("Roll (Y)", JavaUtil.formatNumber(orientation.getRoll(AngleUnit.DEGREES), 2));
                // Display angular velocity.
                telemetry.addData("Yaw (Z) velocity", JavaUtil.formatNumber(angularVelocity.zRotationRate, 2));
                telemetry.addData("Pitch (X) velocity", JavaUtil.formatNumber(angularVelocity.xRotationRate, 2));
                telemetry.addData("Roll (Y) velocity", JavaUtil.formatNumber(angularVelocity.yRotationRate, 2));
                telemetry.update();
                // Display yaw, pitch, and roll.
            }
        }
    }
}